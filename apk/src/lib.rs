mod display;
mod egl;
mod net;

use android_activity::{InputStatus, MainEvent, PollEvent};
use log::{error, info, warn};
use openxr as xr;
use std::{
    path::Path,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

const PROFILE: &str = "/interaction_profiles/bytedance/pico4s_controller";
const GL_RGBA8: u32 = 0x8058;

struct Button {
    action: xr::Action<bool>,
    hand: usize,
    bit: u32,
}

struct EyeSwapchain {
    handle: xr::Swapchain<xr::OpenGlEs>,
    images: Vec<u32>,
}

#[no_mangle]
pub fn android_main(app: android_activity::AndroidApp) {
    android_logger::init_once(
        android_logger::Config::default()
            .with_tag("PicoPose")
            .with_max_level(log::LevelFilter::Info),
    );
    let running = Arc::new(AtomicBool::new(true));
    let activity_ready = Arc::new(AtomicBool::new(false));
    let worker_running = running.clone();
    let worker_ready = activity_ready.clone();
    let worker = thread::spawn(move || {
        while worker_running.load(Ordering::Relaxed) && !worker_ready.load(Ordering::Relaxed) {
            thread::sleep(Duration::from_millis(10));
        }
        let raw_vm = ndk_context::android_context().vm();
        let vm = unsafe { jni::JavaVM::from_raw(raw_vm.cast()) }.ok();
        let _guard = vm.as_ref().and_then(|vm| vm.attach_current_thread().ok());
        if let Err(e) = run(worker_running) {
            error!("{e}");
        }
    });
    let mut resumed = false;
    let mut window = false;
    let mut focused = false;
    while running.load(Ordering::Relaxed) {
        app.poll_events(Some(Duration::from_millis(100)), |event| match event {
            PollEvent::Main(MainEvent::Resume { .. }) => resumed = true,
            PollEvent::Main(MainEvent::Pause) => resumed = false,
            PollEvent::Main(MainEvent::InitWindow { .. }) => window = true,
            PollEvent::Main(MainEvent::TerminateWindow { .. }) => window = false,
            PollEvent::Main(MainEvent::GainedFocus) => focused = true,
            PollEvent::Main(MainEvent::LostFocus) => focused = false,
            PollEvent::Main(MainEvent::Destroy) => running.store(false, Ordering::Relaxed),
            PollEvent::Main(MainEvent::InputAvailable) => {
                if let Ok(mut events) = app.input_events_iter() {
                    while events.next(|_| InputStatus::Unhandled) {}
                }
            }
            _ => {}
        });
        if resumed && window && focused {
            activity_ready.store(true, Ordering::Relaxed);
        }
    }
    worker.join().ok();
}

fn run(running: Arc<AtomicBool>) -> Result<(), String> {
    let entry = unsafe { xr::Entry::load_from(Path::new("libopenxr_loader.so")) }
        .map_err(|e| format!("load OpenXR: {e:?}"))?;
    entry
        .initialize_android_loader()
        .map_err(|e| format!("init OpenXR loader: {e:?}"))?;
    let available = entry
        .enumerate_extensions()
        .map_err(|e| format!("extensions: {e:?}"))?;
    if !available.khr_opengl_es_enable {
        return Err("XR_KHR_opengl_es_enable unavailable".into());
    }
    let mut extensions = xr::ExtensionSet::default();
    extensions.khr_android_create_instance = true;
    extensions.khr_opengl_es_enable = true;
    let instance = entry
        .create_instance(
            &xr::ApplicationInfo {
                application_name: "PICO Head Relative Pose",
                application_version: 1,
                engine_name: "mmhand",
                engine_version: 1,
            },
            &extensions,
            &[],
        )
        .map_err(|e| format!("create instance: {e:?}"))?;
    let system = instance
        .system(xr::FormFactor::HEAD_MOUNTED_DISPLAY)
        .map_err(err)?;
    instance
        .graphics_requirements::<xr::OpenGlEs>(system)
        .map_err(err)?;
    let egl = egl::Context::new()?;
    let (session, mut waiter, mut frames) =
        unsafe { instance.create_session(system, &egl.session_info()) }.map_err(err)?;

    let set = instance
        .create_action_set("poses", "Controller poses", 0)
        .map_err(err)?;
    let left_action = set
        .create_action::<xr::Posef>("left_grip", "Left grip pose", &[])
        .map_err(err)?;
    let right_action = set
        .create_action::<xr::Posef>("right_grip", "Right grip pose", &[])
        .map_err(err)?;
    let profile = instance.string_to_path(PROFILE).map_err(err)?;
    let left_grip = instance
        .string_to_path("/user/hand/left/input/grip/pose")
        .map_err(err)?;
    let right_grip = instance
        .string_to_path("/user/hand/right/input/grip/pose")
        .map_err(err)?;

    let button_specs = [
        ("left_x", "Left X", "/user/hand/left/input/x/click", 0, 0),
        ("left_y", "Left Y", "/user/hand/left/input/y/click", 0, 1),
        (
            "left_menu",
            "Left menu",
            "/user/hand/left/input/menu/click",
            0,
            2,
        ),
        (
            "left_trigger",
            "Left trigger",
            "/user/hand/left/input/trigger/click",
            0,
            3,
        ),
        (
            "left_grip_button",
            "Left grip",
            "/user/hand/left/input/squeeze/click",
            0,
            4,
        ),
        (
            "left_stick",
            "Left stick",
            "/user/hand/left/input/thumbstick/click",
            0,
            5,
        ),
        ("right_a", "Right A", "/user/hand/right/input/a/click", 1, 0),
        ("right_b", "Right B", "/user/hand/right/input/b/click", 1, 1),
        (
            "right_trigger",
            "Right trigger",
            "/user/hand/right/input/trigger/click",
            1,
            2,
        ),
        (
            "right_grip_button",
            "Right grip",
            "/user/hand/right/input/squeeze/click",
            1,
            3,
        ),
        (
            "right_stick",
            "Right stick",
            "/user/hand/right/input/thumbstick/click",
            1,
            4,
        ),
    ];
    let mut buttons = Vec::new();
    let mut button_paths = Vec::new();
    for (name, label, path, hand, bit) in button_specs {
        buttons.push(Button {
            action: set.create_action(name, label, &[]).map_err(err)?,
            hand,
            bit,
        });
        button_paths.push(instance.string_to_path(path).map_err(err)?);
    }
    let mut bindings = vec![
        xr::Binding::new(&left_action, left_grip),
        xr::Binding::new(&right_action, right_grip),
    ];
    bindings.extend(
        buttons
            .iter()
            .zip(button_paths)
            .map(|(button, path)| xr::Binding::new(&button.action, path)),
    );
    instance
        .suggest_interaction_profile_bindings(profile, &bindings)
        .map_err(err)?;
    session.attach_action_sets(&[&set]).map_err(err)?;
    let left_space = left_action
        .create_space(session.clone(), xr::Path::NULL, xr::Posef::IDENTITY)
        .map_err(err)?;
    let right_space = right_action
        .create_space(session.clone(), xr::Path::NULL, xr::Posef::IDENTITY)
        .map_err(err)?;
    let view_space = session
        .create_reference_space(xr::ReferenceSpaceType::VIEW, xr::Posef::IDENTITY)
        .map_err(err)?;
    let local_space = session
        .create_reference_space(xr::ReferenceSpaceType::LOCAL, xr::Posef::IDENTITY)
        .map_err(err)?;
    let mut swaps = make_swapchains(&session)?;
    let status_images = display::Images::new();

    let mut events = xr::EventDataBuffer::new();
    let mut active = false;
    let mut sender = net::Sender::new();
    info!("ready: VIEW-relative poses -> 127.0.0.1:29761");
    while running.load(Ordering::Relaxed) {
        while let Some(event) = instance.poll_event(&mut events).map_err(err)? {
            if let xr::Event::SessionStateChanged(e) = event {
                match e.state() {
                    xr::SessionState::READY => {
                        session
                            .begin(xr::ViewConfigurationType::PRIMARY_STEREO)
                            .map_err(err)?;
                        active = true;
                    }
                    xr::SessionState::STOPPING => {
                        active = false;
                        session.end().map_err(err)?;
                    }
                    xr::SessionState::EXITING | xr::SessionState::LOSS_PENDING => return Ok(()),
                    _ => {}
                }
            }
        }
        if !active {
            thread::sleep(Duration::from_millis(50));
            continue;
        }
        let state = match waiter.wait() {
            Ok(v) => v,
            Err(e) => {
                warn!("wait: {e:?}");
                continue;
            }
        };
        if frames.begin().is_err() {
            continue;
        }
        session.sync_actions(&[(&set).into()]).ok();
        let flags =
            xr::SpaceLocationFlags::POSITION_VALID | xr::SpaceLocationFlags::ORIENTATION_VALID;
        let locate = |space: &xr::Space| {
            space
                .locate(&local_space, state.predicted_display_time)
                .ok()
                .filter(|v| v.location_flags.contains(flags))
                .map(|v| v.pose)
        };
        let head = locate(&view_space);
        let left = head.zip(locate(&left_space)).map(|(h, c)| relative(h, c));
        let right = head.zip(locate(&right_space)).map(|(h, c)| relative(h, c));
        let (left_buttons, right_buttons) = button_masks(&buttons, &session);
        let connected = sender.send(left, right, left_buttons, right_buttons);
        end_frame(
            &session,
            &mut frames,
            &local_space,
            &mut swaps,
            state.predicted_display_time,
            connected,
            &status_images,
        )
        .ok();
    }
    Ok(())
}

fn err(e: impl std::fmt::Debug) -> String {
    format!("{e:?}")
}

fn relative(head: xr::Posef, controller: xr::Posef) -> xr::Posef {
    let inv = xr::Quaternionf {
        x: -head.orientation.x,
        y: -head.orientation.y,
        z: -head.orientation.z,
        w: head.orientation.w,
    };
    let delta = xr::Vector3f {
        x: controller.position.x - head.position.x,
        y: controller.position.y - head.position.y,
        z: controller.position.z - head.position.z,
    };
    xr::Posef {
        orientation: quat_mul(inv, controller.orientation),
        position: rotate(inv, delta),
    }
}

fn quat_mul(a: xr::Quaternionf, b: xr::Quaternionf) -> xr::Quaternionf {
    xr::Quaternionf {
        x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    }
}

fn rotate(q: xr::Quaternionf, v: xr::Vector3f) -> xr::Vector3f {
    let u = xr::Vector3f {
        x: q.x,
        y: q.y,
        z: q.z,
    };
    let dot_uv = u.x * v.x + u.y * v.y + u.z * v.z;
    let dot_uu = u.x * u.x + u.y * u.y + u.z * u.z;
    let cross = xr::Vector3f {
        x: u.y * v.z - u.z * v.y,
        y: u.z * v.x - u.x * v.z,
        z: u.x * v.y - u.y * v.x,
    };
    xr::Vector3f {
        x: 2.0 * dot_uv * u.x + (q.w * q.w - dot_uu) * v.x + 2.0 * q.w * cross.x,
        y: 2.0 * dot_uv * u.y + (q.w * q.w - dot_uu) * v.y + 2.0 * q.w * cross.y,
        z: 2.0 * dot_uv * u.z + (q.w * q.w - dot_uu) * v.z + 2.0 * q.w * cross.z,
    }
}

fn button_masks(buttons: &[Button], session: &xr::Session<xr::OpenGlEs>) -> (u32, u32) {
    let mut masks = [0, 0];
    for button in buttons {
        let pressed = button
            .action
            .state(session, xr::Path::NULL)
            .map(|state| state.is_active && state.current_state)
            .unwrap_or(false);
        if pressed {
            masks[button.hand] |= 1 << button.bit;
        }
    }
    (masks[0], masks[1])
}

fn make_swapchains(session: &xr::Session<xr::OpenGlEs>) -> Result<[EyeSwapchain; 2], String> {
    let formats = session.enumerate_swapchain_formats().map_err(err)?;
    let format = formats
        .iter()
        .copied()
        .find(|v| *v == GL_RGBA8)
        .or_else(|| formats.first().copied())
        .ok_or("no swapchain format")?;
    let info = xr::SwapchainCreateInfo {
        create_flags: xr::SwapchainCreateFlags::EMPTY,
        usage_flags: xr::SwapchainUsageFlags::COLOR_ATTACHMENT,
        format,
        sample_count: 1,
        width: display::SIZE as u32,
        height: display::SIZE as u32,
        face_count: 1,
        array_size: 1,
        mip_count: 1,
    };
    let make = || -> Result<EyeSwapchain, String> {
        let handle = session.create_swapchain(&info).map_err(err)?;
        let images = handle.enumerate_images().map_err(err)?;
        Ok(EyeSwapchain { handle, images })
    };
    Ok([make()?, make()?])
}

fn end_frame(
    session: &xr::Session<xr::OpenGlEs>,
    frames: &mut xr::FrameStream<xr::OpenGlEs>,
    space: &xr::Space,
    swaps: &mut [EyeSwapchain; 2],
    time: xr::Time,
    connected: bool,
    status_images: &display::Images,
) -> Result<(), String> {
    let (_, views) = session
        .locate_views(xr::ViewConfigurationType::PRIMARY_STEREO, time, space)
        .map_err(err)?;
    if views.len() != 2 {
        return frames
            .end(time, xr::EnvironmentBlendMode::OPAQUE, &[])
            .map_err(err);
    }
    for swap in swaps.iter_mut() {
        let index = swap.handle.acquire_image().map_err(err)? as usize;
        swap.handle
            .wait_image(xr::Duration::INFINITE)
            .map_err(err)?;
        status_images.upload(swap.images[index], connected);
        swap.handle.release_image().map_err(err)?;
    }
    let rect = xr::Rect2Di {
        offset: xr::Offset2Di { x: 0, y: 0 },
        extent: xr::Extent2Di {
            width: display::SIZE as i32,
            height: display::SIZE as i32,
        },
    };
    let pv = [0, 1].map(|i| {
        xr::CompositionLayerProjectionView::new()
            .pose(views[i].pose)
            .fov(views[i].fov)
            .sub_image(
                xr::SwapchainSubImage::new()
                    .swapchain(&swaps[i].handle)
                    .image_rect(rect),
            )
    });
    let layer = xr::CompositionLayerProjection::new()
        .space(space)
        .views(&pv);
    frames
        .end(time, xr::EnvironmentBlendMode::OPAQUE, &[&layer])
        .map_err(err)
}
