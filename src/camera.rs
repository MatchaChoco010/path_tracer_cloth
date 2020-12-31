use na::{Rotation3, Vector2, Vector3};
use nalgebra as na;
use rand::distributions::{Distribution, Uniform};

use super::ray::Ray;

// 適当なフィルムサイズ。
// 32mm
const FILM_SIZE: f64 = 0.032;

/// カメラのタイプ。
/// ピンホールカメラとthinレンズモデルのカメラ。
pub enum CameraType {
    Pinhole,
    Lens,
}

/// カメラの構造体。
/// カメラの姿勢やフィルムとレンズの距離、レンズの半径などの値を保持する。
pub struct Camera {
    lens_radius: f64,
    focusing_distance: f64,
    film_distance: f64,
    camera_type: CameraType,
    up: Vector3<f64>,
    view_dir: Vector3<f64>,
    position: Vector3<f64>,
}

impl Camera {
    /// デフォルトでカメラを作成する。
    /// フィルムの距離は画角45度になるように指定している。
    pub fn new() -> Self {
        Self {
            lens_radius: 0.025,
            focusing_distance: 1.0,
            film_distance: (FILM_SIZE / 2.0) / (45.0f64.to_radians() / 2.0).tan(),
            camera_type: CameraType::Pinhole,
            up: Vector3::new(0.0, 1.0, 0.0),
            view_dir: Vector3::new(0.0, 0.0, -1.0),
            position: Vector3::new(0.0, 0.0, 0.0),
        }
    }

    pub fn set_lens_radius(&mut self, lens_radius: f64) {
        self.lens_radius = lens_radius;
    }

    pub fn set_focusing_distance(&mut self, focusing_distance: f64) {
        self.focusing_distance = focusing_distance;
    }

    pub fn set_field_of_view(&mut self, field_of_view: f64) {
        self.film_distance = (FILM_SIZE / 2.0) / (field_of_view / 2.0).tan();
    }

    pub fn set_camera_type(&mut self, camera_type: CameraType) {
        self.camera_type = camera_type;
    }

    pub fn camera_type(&self) -> &CameraType {
        &self.camera_type
    }

    /// カメラの位置を決める
    pub fn set_position(&mut self, position: Vector3<f64>) {
        self.position = position;
    }

    /// カメラの角度をオイラー角で決める。
    pub fn set_eular_angle(&mut self, roll: f64, pitch: f64, yaw: f64) {
        let up = Vector3::new(0.0, 1.0, 0.0);
        let view_dir = Vector3::new(0.0, 0.0, -1.0);
        let rot = Rotation3::from_euler_angles(pitch, yaw, roll);
        self.up = rot.transform_vector(&up);
        self.view_dir = rot.transform_vector(&view_dir);
    }

    /// x/yで指定したピクセルからのrayを取得する。
    /// アンチエイリアスのためにdx/dyを使って位置を微妙にずらしている。
    /// ピンホールカメラならばそのまま、レンズカメラの場合はレンズを通る一つのRayを返している。
    /// レンズカメラはその都合上、ピンホールカメラより大量のサンプル数が必要になるが、
    /// フォーカスが外れた位置のボケなどが表現できる利点がある。
    pub fn get_eye_ray(&self, x: u32, y: u32, res_x: u32, res_y: u32) -> Ray {
        let w = self.view_dir.normalize();
        let u = self.up.cross(&w).normalize();
        let v = w.cross(&u);

        let aspect_ratio = res_x as f64 / res_y as f64;

        let mut rng = rand::thread_rng();
        let uniform = Uniform::new(0.0, 1.0);
        let (r1, r2): (f64, f64) = (
            uniform.sample(&mut rng) * 2.0,
            uniform.sample(&mut rng) * 2.0,
        );
        let dx = if r1 < 1.0 {
            r1.sqrt() - 1.0
        } else {
            1.0 - (2.0 - r1).sqrt()
        };
        let dy = if r2 < 1.0 {
            r2.sqrt() - 1.0
        } else {
            1.0 - (2.0 - r2).sqrt()
        };
        let u_pos = -((x as f64 + 0.5 + dx) / res_x as f64 - 0.5);
        let v_pos = -((y as f64 + 0.5 + dy) / res_y as f64 - 0.5);

        match self.camera_type {
            CameraType::Pinhole => {
                let pixel_pos = self.position
                    + (aspect_ratio * FILM_SIZE * u_pos) * u
                    + (FILM_SIZE * v_pos) * v
                    + self.film_distance * w;
                Ray {
                    o: pixel_pos,
                    d: (pixel_pos - self.position).normalize(),
                }
            }
            CameraType::Lens => {
                let pixel_pos = self.position
                    + (aspect_ratio * FILM_SIZE * u_pos) * -u
                    + (FILM_SIZE * v_pos) * -v
                    + self.film_distance * -w;

                let pinhole_dir = (pixel_pos - self.position).normalize();
                let focusing_distance_point = pinhole_dir * self.focusing_distance
                    / pinhole_dir.dot(&self.view_dir.normalize())
                    + pixel_pos;

                let uniform = Uniform::new(-1.0, 1.0);
                let point = loop {
                    let point = Vector2::new(uniform.sample(&mut rng), uniform.sample(&mut rng));
                    if point.norm() < 1.0 {
                        break point;
                    }
                };
                let point = point * self.lens_radius;
                let point = point.x * u + point.y * v + self.position;

                Ray {
                    o: point,
                    d: (focusing_distance_point - point).normalize(),
                }
            }
        }
    }
}
