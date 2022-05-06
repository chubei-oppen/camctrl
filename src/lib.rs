//! Camera controlling math.

#![warn(
    elided_lifetimes_in_paths,
    explicit_outlives_requirements,
    keyword_idents,
    macro_use_extern_crate,
    meta_variable_misuse,
    missing_abi,
    missing_docs,
    missing_debug_implementations,
    non_ascii_idents,
    noop_method_call,
    pointer_structural_match,
    trivial_casts,
    trivial_numeric_casts,
    unsafe_op_in_unsafe_fn,
    unused_import_braces,
    unused_qualifications,
    unused_results,
    variant_size_differences
)]

pub use glam::Vec2;

use rbt3::Quat;
use rbt3::Rbt;
use rbt3::Vec3;
use rbt3::Vec4;

#[derive(Debug, Clone, PartialEq)]
struct InitialState {
    position: Vec2,
    pose: Rbt,
}

/// Supported camera projection models.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Projection {
    /// Symmetrical pinhole projection.
    /// 
    /// The principal optical axis goes through the viewport center.
    SymmetricalPinhole {
        /// Field of view in the `y` direction.
        fov_y_in_radians: f32
    },
}

impl Projection {
    fn unproject(&self, position: Vec2, viewport_size: Vec2, depth: f32) -> Vec3 {
        match self {
            Self::SymmetricalPinhole { fov_y_in_radians } => {
                let half_frustum_height = (fov_y_in_radians * 0.5).tan();
                let half_frustum_width = half_frustum_height / viewport_size.y * viewport_size.x;

                let x_ndc = (position.x / viewport_size.x) * 2.0 - 1.0;
                let y_ndc = 1.0 - (position.y / viewport_size.y) * 2.0;
                let direction = Vec3::new(
                    x_ndc * half_frustum_width,
                    y_ndc * half_frustum_height,
                    -1.0,
                );
                let direction = direction.normalize();
                direction * depth
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
enum State {
    Immediate,
    Rotating(InitialState),
    Panning(InitialState),
}

/// A utility class for manipulating a 3D camera.
///
/// User constructs `CameraControl` by specifying the focus distance,
/// then feeds UI events to this class by calling its methods.
/// The methods will return new camera pose calculated based on UI operation.
/// 
/// ## Operation Mode
/// 
/// `CameraControl` has two modes: `remember` mode and `immediate` mode.
/// 
/// In `remember` mode, user first specifies the initial state (which `CameraControl` `remember`s),
/// then feeds arbitary number events and gets camera poses. The camera pose is irrelavant with
/// the pointer's moving path, only relavant with the final position.
/// 
/// In `immediate` mode, user feeds an event and gets a new pose. No state is remembered.
/// 
/// Rotation and panning operate in `remember` mode, zomming in `immediate` mode.
/// 
/// ## Convention
///
/// This class assumes the camera looks at its `-z` axis,
/// while `x` axis points right, `y` axis points up.
///
/// The UI events coordinate should be under the coordinate system with origin
/// at the window's top left corner, `x` axis pointing right, `y` axis pointing down,
/// unit length being `1` pixel.
#[derive(Debug, Clone)]
pub struct CameraControl {
    state: State,
    focus_distance: f32,
}

impl CameraControl {
    /// Creates a new `CameraControl` with given focus distance.
    ///
    /// # Arguments:
    ///
    /// * `focus_distance` - A distance controlling rotation and panning behaviour.
    ///
    /// When rotating, the rotation center is the point that's straightly in front of the camera
    /// at the given distance.
    ///
    /// When panning, the point that projects to the panning starting point and at the given
    /// distance to the camera `xy` plane will stick to the pointer.
    pub fn new(focus_distance: f32) -> Self {
        CameraControl {
            state: State::Immediate,
            focus_distance,
        }
    }

    /// Puts the `CameraControl` in `rotating` state (`remember` mode).
    ///
    /// # Arguments:
    /// * `initial_position` - Position of the pointer at the start of the rotation operation.
    /// * `initial_pose` - Pose of the camera at the start of the rotation operation.
    pub fn start_rotating(&mut self, initial_position: Vec2, initial_pose: Rbt) {
        self.state = State::Rotating(InitialState {
            position: initial_position,
            pose: initial_pose,
        });
    }

    /// Puts the `CameraControl` in `panning` state (`remember` mode).
    ///
    /// # Arguments:
    /// * `initial_position` - Position of the pointer at the start of the panning operation.
    /// * `initial_pose` - Pose of the camera at the start of the panning operation.
    pub fn start_panning(&mut self, initial_position: Vec2, initial_pose: Rbt) {
        self.state = State::Panning(InitialState {
            position: initial_position,
            pose: initial_pose,
        });
    }

    /// Calculates the camera pose when the pointer moves to a new position.
    ///
    /// # Arguments:
    /// * `position` - New position of the pointer.
    /// * `projection`: Camera's projection model.
    /// * `viewport_size`: The viewport's size.
    ///
    /// Returns `None` if the `CameraControl` is in `immediate` mode, new camera pose otherwise.
    pub fn calculate_new_pose(
        &self,
        position: Vec2,
        projection: Projection,
        viewport_size: Vec2,
    ) -> Option<Rbt> {
        match &self.state {
            State::Immediate => None,
            State::Rotating(initial_state) => Some(calc_arcball_pose(
                &initial_state.pose,
                self.focus_distance,
                viewport_size,
                initial_state.position,
                position,
            )),
            State::Panning(initial_state) => Some(calc_pan_pose(
                &initial_state.pose,
                projection,
                self.focus_distance,
                viewport_size,
                initial_state.position,
                position,
            )),
        }
    }

    /// Puts the `CameraControl` in `immediate` mode and zooms it.
    ///
    /// Zooming speed is proportional to the focus distance,
    /// so zooming is slower when camera is nearer to its focus.
    ///
    /// Zooming will modify the focus distance so that the focus point stays unchanged.
    ///
    /// # Arguments:
    /// * `pose` - Camera pose before zooming.
    /// * `step` - Number of steps to zoom. Negative value makes the camera go forward, positive backward.
    ///
    /// Returns the new camera pose.
    pub fn zoom(&mut self, pose: Rbt, step: f32) -> Rbt {
        self.state = State::Immediate;
        let move_length = if step > 0.0 {
            self.focus_distance * step * BACKWARD_SPEED
        } else {
            self.focus_distance * step * BACKWARD_SPEED / (1.0 + BACKWARD_SPEED)
        };
        let translation = Rbt::from_t(Vec3::new(0.0, 0.0, move_length));
        let new_pose = pose * translation;
        self.focus_distance += move_length;
        new_pose
    }

    /// Gets current focus distance.
    pub fn focus_distance(&self) -> f32 {
        self.focus_distance
    }

    /// Puts the `CameraControl` in `immediate` mode and sets the focus distance.
    pub fn set_focus_distance(&mut self, focus_distance: f32) {
        self.state = State::Immediate;
        self.focus_distance = focus_distance;
    }

    /// Returns if the `CameraControl` is in `rotating` state.
    pub fn is_rotating(&self) -> bool {
        if let State::Rotating(_) = self.state {
            true
        } else {
            false
        }
    }

    /// Returns if the `CameraControl` is in `panning` state.
    pub fn is_panning(&self) -> bool {
        if let State::Panning(_) = self.state {
            true
        } else {
            false
        }
    }

    /// Puts the `CameraControl` in `immediate` mode.
    pub fn stop_rotating_or_panning(&mut self) {
        self.state = State::Immediate;
    }
}

const BACKWARD_SPEED: f32 = 0.1;

fn calc_arcball_pose(
    initial_pose: &Rbt,
    distance: f32,
    viewport_size: Vec2,
    from: Vec2,
    to: Vec2,
) -> Rbt {
    let from = map_point_to_direction(viewport_size, from);
    let to = map_point_to_direction(viewport_size, to);
    let rotation = from * to.inverse();
    let rotation = Rbt::from_r(rotation);
    let target = (initial_pose * Vec4::new(0.0, 0.0, -distance, 1.0)).truncate();
    let respect = Rbt::from_t_r(target, initial_pose.rotation);
    Rbt::do_m_to_o_wrt_a(&rotation, initial_pose, &respect)
}

fn map_point_to_direction(viewport_size: Vec2, position: Vec2) -> Quat {
    let x = position.x - viewport_size.x * 0.5;
    let y = viewport_size.y * 0.5 - position.y;
    let r = viewport_size.x.min(viewport_size.y) * 0.6;
    // Bell's trackball https://curis.ku.dk/ws/files/38552161/01260772.pdf (43)
    let x2py2 = x * x + y * y;
    let r2 = r * r;
    let p = if x2py2 <= r2 * 0.5 {
        Vec3::new(x, y, (r2 - x2py2).sqrt())
    } else {
        Vec3::new(x, y, r2 * 0.5 / x2py2.sqrt())
    };
    let p = p.normalize();
    Quat::from_xyzw(p.x, p.y, p.z, 0.0)
}

fn calc_pan_pose(
    initial_pose: &Rbt,
    projection: Projection,
    distance: f32,
    viewport_size: Vec2,
    from: Vec2,
    to: Vec2,
) -> Rbt {
    let from = projection.unproject(from, viewport_size, -distance);
    let to = projection.unproject(to, viewport_size, -distance);
    let translation = Rbt::from_t(to - from);
    let respect = Rbt::from_r(initial_pose.rotation);
    Rbt::do_m_to_o_wrt_a(&translation, initial_pose, &respect)
}
