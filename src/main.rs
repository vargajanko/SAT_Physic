mod aabb_tree;

use std::{f32::consts::PI, ops::Mul};

use bevy::{
    math::Vec3A,
    pbr::wireframe::{Wireframe, WireframePlugin},
    prelude::{system_adapter::new, *},
    render::{
        color,
        settings::{WgpuFeatures, WgpuSettings},
        RenderPlugin,
    },
};
use itertools::Itertools;
use sat_physic::{
    capsule_capsule_overlap, capsule_convex_solver, convex_convex_solver, local_transform,
    sphere_capsule_overlap, sphere_convex_overlap, sphere_convex_solver, Capsule, ConvexHull,
    Sphere,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(RenderPlugin {
            wgpu_settings: WgpuSettings {
                features: WgpuFeatures::POLYGON_MODE_LINE,
                ..default()
            },
        }))
        .add_plugin(WireframePlugin)
        .add_startup_system(setup)
        .add_system(collision_check)
        .add_system(move_sphere.before(collision_check))
        .add_system(test_matrix)
        .run();
}

#[derive(Component)]
pub struct DebugSimplex {}

pub fn test_matrix() {
    let mat_a = Mat3::from_cols(
        Vec3::new(0.7, 0.0, 0.7),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(-0.7, 0.0, 0.7),
    );

    let mat_b = Mat3::from_cols(
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        Vec3::new(0.0, 0.0, 1.0),
    );

    let erg = local_transform(&mat_b, &mat_a);
    let erg_vec = erg.mul_vec3(Vec3::new(1.0, 0.0, 0.0));
}

pub fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    //Cube for rendering and physics
    let vertices = vec![
        Vec3::new(0.5, 0.5, 0.5),    //0
        Vec3::new(0.5, 0.5, -0.5),   //1
        Vec3::new(0.5, -0.5, 0.5),   //2
        Vec3::new(0.5, -0.5, -0.5),  //3
        Vec3::new(-0.5, 0.5, 0.5),   //4
        Vec3::new(-0.5, 0.5, -0.5),  //5
        Vec3::new(-0.5, -0.5, 0.5),  //6
        Vec3::new(-0.5, -0.5, -0.5), //7
    ];
    let mut te = Transform::from_xyz(1.0, 0.0, 0.0);
    te.rotate_axis(Vec3::new(0.0, 1.0, 0.0), 45.0);
    commands.spawn((
        ConvexHull::new(
            Vec3::new(1.0, 0.0, 0.0),
            vertices.clone(),
            Vec3::new(0.0, 1.0, 0.0),
            45.0,
        ),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
            material: materials.add(Color::rgba(0.8, 0.7, 0.6, 1.0).into()),
            transform: te,
            ..default()
        },
        Wireframe,
    ));

    commands.spawn((PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1 })),
        material: materials.add(Color::rgb(0.0, 0.0, 1.0).into()),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    },));

    commands.spawn((
        ConvexHull::new(
            Vec3::new(2.0, 0.0, 0.0),
            vertices,
            Vec3::new(1.0, 0.0, 0.0),
            0.0,
        ),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
            material: materials.add(Color::rgba(0.8, 0.7, 0.6, 1.0).into()),
            transform: Transform::from_xyz(2.0, 0.0, 0.0),
            ..default()
        },
    ));

    commands.spawn((PbrBundle {
        mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1 })),
        material: materials.add(Color::rgb(0.0, 0.0, 1.0).into()),
        transform: Transform::from_xyz(0.0, 0.0, 0.0),
        ..default()
    },));

    //Capsule for rendering and collision
    commands.spawn((
        Capsule {
            radius: 0.4,
            upper_center: Vec3 {
                x: -std::f32::consts::FRAC_1_SQRT_2,
                y: std::f32::consts::FRAC_1_SQRT_2,
                z: 0.0,
            },
            lower_center: Vec3 {
                x: std::f32::consts::FRAC_1_SQRT_2,
                y: -std::f32::consts::FRAC_1_SQRT_2,
                z: 0.0,
            },
        },
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Capsule {
                radius: 0.4,
                rings: 5,
                depth: 2.0,
                latitudes: 20,
                longitudes: 20,
                uv_profile: default(),
            })),
            material: materials.add(Color::rgba(0.8, 0.7, 0.6, 0.0).into()),
            transform: Transform {
                translation: Vec3::new(0.0, 0.0, 0.0),
                rotation: Quat::from_rotation_z(PI / 4.0),
                scale: Vec3::new(1.0, 1.0, 1.0),
            },
            ..default()
        },
        Wireframe,
    ));
    /*
        commands.spawn((
            Capsule {
                radius: 1.0,
                upper_center: Vec3 {
                    x: 1.0,
                    y: 1.0,
                    z: 0.0,
                },
                lower_center: Vec3 {
                    x: 1.0,
                    y: -1.0,
                    z: 0.0,
                },
            },
            PbrBundle {
                mesh: meshes.add(Mesh::from(shape::Capsule {
                    radius: 1.0,
                    rings: 5,
                    depth: 2.0,
                    latitudes: 20,
                    longitudes: 20,
                    uv_profile: default(),
                })),
                material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
                transform: Transform::from_xyz(1.0, 0.0, 0.0),
                ..default()
            },
        ));
    */
    //Sphere for rendering and for collision
    /*commands.spawn((
            PbrBundle {
                mesh: meshes.add(
                    shape::Icosphere {
                        radius: 0.5,
                        subdivisions: 4,
                    }
                    .try_into()
                    .unwrap(),
                ),
                material: materials.add(Color::rgba(0.8, 0.7, 0.6, 0.0).into()),
                transform: Transform::from_xyz(-0.025533473, -0.6021381, 0.30380815),
                ..default()
            },
            Sphere {
                center: Vec3::new(-0.025533473, -0.6021381, 0.30380815),
                radius: 0.5,
            },
            Wireframe,
        ));
    */
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 1500.0,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(4.0, 8.0, 4.0),
        ..default()
    });

    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(1.5, 0.0, 2.0)
            .looking_at(Vec3::new(1.5, -0.0, 0.0), Vec3::Y),
        ..default()
    });
}

pub fn collision_check(
    mut commands: Commands,
    spheres: Query<&Capsule>,
    mut capsules: Query<(&mut ConvexHull, &mut Transform), Without<DebugSimplex>>,
    debug_cubes: Query<Entity, With<DebugSimplex>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    /*for sp_collision in spheres.into_iter() {
        for cap_collision in capsules.into_iter() {
            let collide = sphere_capsule_overlap(sp_collision, cap_collision);
            //println!("Does sphere and capsule collide? {}", collide);
        }
    }*/

    /*for capa_collision in capsules.into_iter().combinations(2) {
        let collide = capsule_capsule_overlap(capa_collision[0], capa_collision[1]);
        println!("Does capsule and capsule collide? {}", collide);
    }*/

    for cub in debug_cubes.iter() {
        commands.entity(cub).despawn();
    }

    let mut move_: Vec3 = Vec3::default();

    for (convex1, convex2) in capsules
        .iter()
        .step_by(2)
        .zip(capsules.iter().skip(1).step_by(2))
    {
        let manifold = convex_convex_solver(&convex1.0, &convex2.0);
        if manifold.point_count > 0 {
            println!(
                "collision poins: {}, distance: {}, normal dir: {}",
                manifold.point_count, manifold.points[0].penetration, manifold.normal
            );
            move_ = manifold.points[0].penetration * manifold.normal;
            for i in 0..manifold.point_count {
                println!("index; {}", i);
                commands.spawn((
                    PbrBundle {
                        mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1 })),
                        material: materials.add(Color::rgb(0.0, 1.0, 0.0).into()),
                        transform: Transform::from_translation(manifold.points[i].position),
                        ..default()
                    },
                    DebugSimplex {},
                ));
            }
        }
    }
    for mut convex1 in capsules.iter_mut().take(1) {
        convex1.0.center = convex1.0.center + move_;
        convex1.1.translation = convex1.1.translation + move_;
    }
    /*for sp_collision in spheres.into_iter() {
        for cap_collision in capsules.iter() {
            //cap_collision.create_planes_box();
            //let collide = sphere_convex_overlap(sp_collision, cap_collision);
            //draw the found simplex
            let manifold = capsule_convex_solver(sp_collision, cap_collision);

            /* if manifold.point_count == 1 {
                println!("deep penetration");
                println!("normal {}", manifold.normal);
            } else if manifold.point_count == 2 {
                println!("shallow penetration");
            } else {
                println!("no penetration");
            }*/
            println!(
                "contact point: {}, minimum distance: {}",
                manifold.points[0].position, manifold.points[0].penetration
            );
            if manifold.points[0].penetration < 0.0 {
                println!("shallow pene");
            }
            /*commands.spawn((
                PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
                    material: materials.add(Color::rgba(1.0, 0.0, 0.0, 0.0).into()),
                    transform: Transform::from_translation(
                        cap_collision.center - sp_collision.center,
                    ),
                    ..default()
                },
                DebugSimplex {},
                Wireframe,
            ));*/

            commands.spawn((
                PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1 })),
                    material: materials.add(Color::rgb(0.0, 1.0, 0.0).into()),
                    transform: Transform::from_translation(sp_collision.lower_center),
                    ..default()
                },
                DebugSimplex {},
            ));

            commands.spawn((
                PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1 })),
                    material: materials.add(Color::rgb(0.0, 1.0, 0.0).into()),
                    transform: Transform::from_translation(sp_collision.upper_center),
                    ..default()
                },
                DebugSimplex {},
            ));

            commands.spawn((
                PbrBundle {
                    mesh: meshes.add(Mesh::from(shape::Cube { size: 0.2 })),
                    material: materials.add(Color::rgb(0.0, 1.0, 1.0).into()),
                    transform: Transform::from_translation(manifold.points[0].position),
                    ..default()
                },
                DebugSimplex {},
            ));
            let mut color_code = [Vec3::default(); 4];
            color_code[0] = Vec3::new(1.0, 0.0, 0.0);
            color_code[1] = Vec3::new(1.0, 1.0, 1.0);
            color_code[2] = Vec3::new(0.0, 0.0, 1.0);
            color_code[3] = Vec3::new(1.0, 1.0, 1.0);
            /*for w in 0..collide.1 {
                commands.spawn((
                    PbrBundle {
                        mesh: meshes.add(Mesh::from(shape::Cube { size: 0.1 })),
                        material: materials.add(
                            Color::rgb(color_code[w].x, color_code[w].y, color_code[w].z).into(),
                        ),
                        transform: Transform::from_translation(collide.2[w]),
                        ..default()
                    },
                    DebugSimplex {},
                ));
            } */
        }
    }*/
}

pub fn move_sphere(
    keyboard_input: Res<Input<KeyCode>>,
    mut spheres: Query<(&mut ConvexHull, &mut Transform)>,
    time: Res<Time>,
) {
    let mut direction = Vec3::ZERO;

    if keyboard_input.pressed(KeyCode::Left) {
        direction += Vec3::new(-0.1, 0.0, 0.0);
    }
    if keyboard_input.pressed(KeyCode::Right) {
        direction += Vec3::new(0.1, 0.0, 0.0);
    }
    if keyboard_input.pressed(KeyCode::Up) {
        direction += Vec3::new(0.0, 0.1, 0.0);
    }
    if keyboard_input.pressed(KeyCode::Down) {
        direction += Vec3::new(0.0, -0.1, 0.0);
    }
    if keyboard_input.pressed(KeyCode::I) {
        direction += Vec3::new(0.0, 0.0, 0.1);
    }
    if keyboard_input.pressed(KeyCode::K) {
        direction += Vec3::new(0.0, 0.0, -0.1);
    }

    direction *= time.delta_seconds();
    let mut x = 0;
    for (mut col, mut ren) in &mut spheres {
        if x == 0 {
            col.center += direction;
            //col.lower_center += direction;
            ren.translation += direction;
            x += 1;
        }
    }
}
