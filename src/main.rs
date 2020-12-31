#![allow(unused_variables)]

use std::sync::Arc;

use anyhow::Result;
use na::Vector3;
use nalgebra as na;

use path_tracer_cloth::{Camera, Material, Scene, TriangleMesh};

fn make_scene() -> Result<(Camera, Scene)> {
    let mut cam = Camera::new();
    cam.set_field_of_view(90.0f64.to_radians());
    cam.set_position(Vector3::new(0.0, 10.0, 20.0));
    cam.set_eular_angle(0.0, 0.0, 0.0);
    let cam = cam;

    let mut scene = Scene::new();

    let wall_material = Arc::new(Material::new_lambert(Vector3::new(0.75, 0.75, 0.75)));
    // let wall_red_material = Arc::new(Material::new_lambert(Vector3::new(0.75, 0.0, 0.0)));
    // let wall_green_material = Arc::new(Material::new_lambert(Vector3::new(0.0, 0.75, 0.0)));
    let light_material = Arc::new(Material::new_emissive(Vector3::new(1.0, 1.0, 1.0), 20.0));

    let a_linen_plain_material = Arc::new(Material::new_cloth(
        1.46,
        Vector3::new(0.2, 0.8, 1.0) * 0.3,
        0.3,
        12.0f64.to_radians(),
        24.0f64.to_radians(),
        0.33,
        vec![-25.0, 25.0],
        1.46,
        Vector3::new(0.2, 0.8, 1.0) * 0.3,
        0.3,
        12.0f64.to_radians(),
        24.0f64.to_radians(),
        0.33,
        vec![-25.0, 25.0],
    ));
    let b_silk_crepe_de_chine_material = Arc::new(Material::new_cloth(
        1.345,
        Vector3::new(1.0, 0.95, 0.05) * 0.12,
        0.2,
        5.0f64.to_radians(),
        10.0f64.to_radians(),
        0.75,
        vec![-35.0, -35.0, 35.0, 35.0],
        1.345,
        Vector3::new(1.0, 0.95, 0.05) * 0.16,
        0.3,
        18.0f64.to_radians(),
        32.0f64.to_radians(),
        0.25,
        vec![0.0],
    ));
    let c_polyester_satin_charmeuse_material = Arc::new(Material::new_cloth(
        1.539,
        Vector3::new(1.0, 0.37, 0.3) * 0.035,
        0.1,
        2.5f64.to_radians(),
        5.0f64.to_radians(),
        0.9,
        vec![-32.0, -32.0, -18.0, 0.0, 0.0, 18.0, 32.0, 32.0],
        1.539,
        Vector3::new(1.0, 0.37, 0.3) * 0.02,
        0.7,
        30.0f64.to_radians(),
        60.0f64.to_radians(),
        0.1,
        vec![0.0],
    ));
    let d_polyester_satin_charmeuse_back_material = Arc::new(Material::new_cloth(
        1.539,
        Vector3::new(1.0, 0.37, 0.3) * 0.035,
        0.1,
        2.5f64.to_radians(),
        5.0f64.to_radians(),
        0.67,
        vec![-30.0, -30.0, 30.0, 30.0, -5.0, -5.0, 5.0, 5.0],
        1.539,
        Vector3::new(1.0, 0.37, 0.3) * 0.02,
        0.7,
        30.0f64.to_radians(),
        60.0f64.to_radians(),
        0.33,
        vec![0.0],
    ));
    let e_silk_shot_material = Arc::new(Material::new_cloth(
        1.345,
        Vector3::new(0.1, 1.0, 0.4) * 0.2,
        0.1,
        4.0f64.to_radians(),
        8.0f64.to_radians(),
        0.86,
        vec![-25.0, -25.0, 25.0, 25.0],
        1.345,
        Vector3::new(1.0, 0.0, 0.1) * 0.6,
        0.1,
        5.0f64.to_radians(),
        10.0f64.to_radians(),
        0.14,
        vec![0.0],
    ));
    let f_velvet_material = Arc::new(Material::new_cloth(
        1.46,
        Vector3::new(0.05, 0.02, 0.0) * 0.3,
        0.1,
        6.0f64.to_radians(),
        12.0f64.to_radians(),
        0.5,
        vec![-90.0, -50.0],
        1.46,
        Vector3::new(0.05, 0.02, 0.0) * 0.3,
        0.1,
        6.0f64.to_radians(),
        12.0f64.to_radians(),
        0.5,
        vec![-90.0, -55.0, 55.0, 90.0],
    ));

    scene.add_mesh(
        TriangleMesh::new("wall-0.obj").build()?,
        Arc::clone(&wall_material),
    );
    scene.add_mesh(
        TriangleMesh::new("wall-1.obj").build()?,
        Arc::clone(&wall_material),
    );
    scene.add_mesh(
        TriangleMesh::new("wall-2.obj").build()?,
        Arc::clone(&wall_material),
    );
    scene.add_mesh(
        TriangleMesh::new("wall-3.obj").build()?,
        Arc::clone(&wall_material),
    );
    scene.add_mesh(
        TriangleMesh::new("wall-4.obj").build()?,
        Arc::clone(&wall_material),
    );
    scene.add_mesh(
        TriangleMesh::new("light.obj").build()?,
        Arc::clone(&light_material),
    );
    scene.add_mesh(
        TriangleMesh::new("cloth.obj").build()?,
        Arc::clone(&e_silk_shot_material),
    );

    let scene = scene.build().unwrap();

    Ok((cam, scene))
}

fn main() -> Result<()> {
    println!("Start Cloth scene.");
    let (cam, scene) = make_scene()?;
    scene.render_image_l_white(&cam, "cloth_e.png", 512, 512, 1.5, 1000);

    Ok(())
}
