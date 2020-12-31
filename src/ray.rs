use na::Vector3;
use nalgebra as na;

/// レイを表現する構造体
pub struct Ray {
    pub o: Vector3<f64>,
    pub d: Vector3<f64>,
}
