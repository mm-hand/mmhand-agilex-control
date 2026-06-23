use openxr as xr;
use std::{
    io::Write,
    net::{SocketAddr, TcpStream},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

const ADDRESS: &str = "127.0.0.1:29761";

pub struct Sender {
    stream: Option<TcpStream>,
    retry_at: Instant,
}

impl Sender {
    pub fn new() -> Self {
        Self {
            stream: None,
            retry_at: Instant::now(),
        }
    }

    pub fn send(
        &mut self,
        left: Option<xr::Posef>,
        right: Option<xr::Posef>,
        left_buttons: u32,
        right_buttons: u32,
    ) -> bool {
        if self.stream.is_none() && Instant::now() >= self.retry_at {
            let addr: SocketAddr = ADDRESS.parse().unwrap();
            self.stream = TcpStream::connect_timeout(&addr, Duration::from_millis(100)).ok();
            if let Some(stream) = &self.stream {
                stream.set_nodelay(true).ok();
                stream
                    .set_write_timeout(Some(Duration::from_millis(20)))
                    .ok();
            } else {
                self.retry_at = Instant::now() + Duration::from_secs(1);
            }
        }
        let Some(stream) = &mut self.stream else {
            return false;
        };
        let t = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();
        let line = format!(
            "{t:.6},{},{},{left_buttons},{right_buttons}\n",
            fields(left),
            fields(right)
        );
        if stream.write_all(line.as_bytes()).is_err() {
            self.stream = None;
            self.retry_at = Instant::now() + Duration::from_secs(1);
        }
        self.stream.is_some()
    }
}

fn fields(pose: Option<xr::Posef>) -> String {
    match pose {
        Some(p) => format!(
            "1,{:.6},{:.6},{:.6},{:.7},{:.7},{:.7},{:.7}",
            p.position.x,
            p.position.y,
            p.position.z,
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w
        ),
        None => "0,0,0,0,0,0,0,1".into(),
    }
}
