use std::sync::Arc;
use std::time::Instant;

use anyhow::Result;
use image::{ImageBuffer, Rgb};
use na::Vector3;
use nalgebra as na;
use rayon::prelude::*;

use super::bvh::BVH;
use super::camera::Camera;
use super::material::Material;
use super::path_trace::*;
use super::triangle::Triangle;
use super::triangle_mesh::TriangleMesh;

/// シーンのビルダー用構造体。
pub struct SceneBuilder {
    meshes: Vec<TriangleMesh>,
    materials: Vec<Arc<Material>>,
}

impl SceneBuilder {
    /// TriangleMeshを追加する
    pub fn add_mesh(&mut self, mesh: TriangleMesh, material: Arc<Material>) {
        self.meshes.push(mesh);
        self.materials.push(material);
    }

    /// TriangleMesh群から三角形群を作成しBVHを構築してシーンを作成する
    pub fn build(self) -> Result<Scene> {
        // meshをArcで包む
        let meshes = self
            .meshes
            .into_iter()
            .map(|m| Arc::new(m))
            .collect::<Vec<Arc<TriangleMesh>>>();

        let mut triangles = vec![];
        for (mesh, material) in meshes.iter().zip(self.materials.iter()) {
            for (i, _) in mesh.v_indices.iter().enumerate() {
                let tri = Triangle::new(i, Arc::clone(mesh), Arc::clone(material));
                triangles.push(tri);
            }
        }
        let bvh = BVH::new(triangles);

        Ok(Scene { bvh })
    }
}

/// BVHを保持するシーン構造体。
pub struct Scene {
    bvh: BVH,
}

impl Scene {
    /// シーンのビルダーオブジェクトを生成する。
    pub fn new() -> SceneBuilder {
        SceneBuilder {
            meshes: vec![],
            materials: vec![],
        }
    }

    pub fn bvh(&self) -> &BVH {
        &self.bvh
    }

    /// シーンをレンダリングする。
    /// レンダリング結果のHDR画像をSDRに落とし込む際にReinhardのトーンマップを利用している。
    pub fn render_image(
        &self,
        cam: &Camera,
        filename: &str,
        width: u32,
        height: u32,
        samples: u32,
    ) {
        self.render_image_l_white(cam, filename, width, height, 1.0, samples)
    }

    /// シーンをレンダリングする。
    /// レンダリング結果のHDR画像をSDRに落とし込む際にReinhardのトーンマップを利用している。
    /// Reinhardのトーンマップのパラメータ、l_whiteを指定できる。
    pub fn render_image_l_white(
        &self,
        cam: &Camera,
        filename: &str,
        width: u32,
        height: u32,
        l_white: f64,
        samples: u32,
    ) {
        let mut img = ImageBuffer::new(width, height);

        println!("Start rendering...");
        let start = Instant::now();

        img.enumerate_pixels_mut()
            .collect::<Vec<(u32, u32, &mut Rgb<u8>)>>()
            .par_iter_mut()
            .for_each(|(x, y, pixel)| {
                let mut rgb = Vector3::zeros();

                for _ in 0..samples {
                    let ray = cam.get_eye_ray(*x, *y, width, height);
                    rgb += path_trace(&ray, self, 0);
                }
                let rgb = rgb / samples as f64;

                // Reinhard
                let r = (rgb.x * (1.0 + rgb.x / l_white.powi(2))) / (1.0 + rgb.x);
                let g = (rgb.y * (1.0 + rgb.y / l_white.powi(2))) / (1.0 + rgb.y);
                let b = (rgb.z * (1.0 + rgb.z / l_white.powi(2))) / (1.0 + rgb.z);

                // gamma correction
                let r = r.powf(1.0 / 2.2);
                let g = g.powf(1.0 / 2.2);
                let b = b.powf(1.0 / 2.2);

                pixel[0] = (r * 255.0).max(0.0).min(255.0) as u8;
                pixel[1] = (g * 255.0).max(0.0).min(255.0) as u8;
                pixel[2] = (b * 255.0).max(0.0).min(255.0) as u8;
            });

        let end = start.elapsed();
        println!(
            "Finish rendering.\nRendering time: {}.{:03}s",
            end.as_secs(),
            end.subsec_millis()
        );

        println!("save image {}...", filename);
        img.save(filename).unwrap();
    }
}
