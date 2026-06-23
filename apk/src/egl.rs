use khronos_egl as egl;
use std::ffi::c_void;

type Egl = egl::DynamicInstance<egl::EGL1_4>;

pub struct Context {
    egl: Egl,
    display: egl::Display,
    config: egl::Config,
    context: egl::Context,
    surface: egl::Surface,
}

impl Context {
    pub fn new() -> Result<Self, String> {
        let egl = unsafe { Egl::load_required() }.map_err(|e| format!("load EGL: {e:?}"))?;
        let display = unsafe { egl.get_display(egl::DEFAULT_DISPLAY) }.ok_or("no EGL display")?;
        egl.initialize(display)
            .map_err(|e| format!("init EGL: {e:?}"))?;
        egl.bind_api(egl::OPENGL_ES_API)
            .map_err(|e| format!("bind GLES: {e:?}"))?;
        let attrs = [
            egl::SURFACE_TYPE,
            egl::PBUFFER_BIT,
            egl::RENDERABLE_TYPE,
            egl::OPENGL_ES3_BIT,
            egl::RED_SIZE,
            8,
            egl::GREEN_SIZE,
            8,
            egl::BLUE_SIZE,
            8,
            egl::ALPHA_SIZE,
            8,
            egl::NONE,
        ];
        let config = egl
            .choose_first_config(display, &attrs)
            .map_err(|e| format!("choose EGL config: {e:?}"))?
            .ok_or("no EGL config")?;
        let context = egl
            .create_context(
                display,
                config,
                None,
                &[egl::CONTEXT_CLIENT_VERSION, 3, egl::NONE],
            )
            .map_err(|e| format!("create EGL context: {e:?}"))?;
        let surface = egl
            .create_pbuffer_surface(
                display,
                config,
                &[egl::WIDTH, 16, egl::HEIGHT, 16, egl::NONE],
            )
            .map_err(|e| format!("create EGL surface: {e:?}"))?;
        egl.make_current(display, Some(surface), Some(surface), Some(context))
            .map_err(|e| format!("activate EGL: {e:?}"))?;
        Ok(Self {
            egl,
            display,
            config,
            context,
            surface,
        })
    }

    pub fn session_info(&self) -> openxr::opengles::SessionCreateInfo {
        openxr::opengles::SessionCreateInfo::Android {
            display: self.display.as_ptr().cast::<c_void>(),
            config: self.config.as_ptr().cast::<c_void>(),
            context: self.context.as_ptr().cast::<c_void>(),
        }
    }
}

impl Drop for Context {
    fn drop(&mut self) {
        self.egl.make_current(self.display, None, None, None).ok();
        self.egl.destroy_surface(self.display, self.surface).ok();
        self.egl.destroy_context(self.display, self.context).ok();
        self.egl.terminate(self.display).ok();
    }
}
