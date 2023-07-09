use std::f32::consts::PI;

use aabb_tree::{MovedAabbs, Tree};
use bevy::{
    diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin},
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
    convex_convex_solver, create_planes_box, local_transform, Capsule, ConvexHull, Sphere,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(RenderPlugin {
            wgpu_settings: WgpuSettings {
                features: WgpuFeatures::POLYGON_MODE_LINE,
                ..default()
            },
        }))
        .init_resource::<Tree>()
        .init_resource::<MovedAabbs>()
        .add_plugin(WireframePlugin)
        .add_startup_system(setup)
        .add_system(collision_broad_phase)
        //.add_system(collision_check)
        .add_plugin(LogDiagnosticsPlugin::default())
        .add_plugin(FrameTimeDiagnosticsPlugin::default())
        .add_system(move_sphere.before(collision_check))
        .add_system(test_matrix)
        .run();
}

#[derive(Component)]
pub struct DebugSimplex {}

#[derive(Component)]
pub struct Collider {
    pub collider_id: usize,
    pub convex_hull: ConvexHull,
}

impl Collider {
    pub fn new(
        position: Vec3,
        vertices: Vec<Vec3>,
        axis: Vec3,
        angle: f32,
        tree: &mut ResMut<Tree>,
    ) -> Self {
        let mut x = ConvexHull {
            center: position,
            vertices: vertices.clone(),
            edges: Default::default(),
            faces: Default::default(),
            planes: create_planes_box(vertices),
            orientation: Quat::from_axis_angle(axis, angle),
        };

        x.create_halfedges();
        println!("{:?}", x.create_aabb());
        Collider {
            collider_id: tree.create_aabb(&x.create_aabb()),
            convex_hull: x,
        }
    }
}

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
    mut tree: ResMut<Tree>,
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

    let position = Vec3::new(1.0, 0.0, 0.0);
    let angle = 45.0;
    let rotation_axis = Vec3::new(0.0, 1.0, 0.0);
    let mut te = Transform::from_translation(position);
    te.rotate_axis(rotation_axis, angle);

    commands.spawn((
        Collider::new(position, vertices.clone(), rotation_axis, angle, &mut tree),
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

    let position = Vec3::new(2.0, 0.0, 0.0);
    let angle = 0.0;
    let rotation_axis = Vec3::new(1.0, 0.0, 0.0);
    let mut te = Transform::from_translation(position);
    te.rotate_axis(rotation_axis, angle);

    commands.spawn((
        Collider::new(position, vertices, rotation_axis, angle, &mut tree),
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Cube { size: 1.0 })),
            material: materials.add(Color::rgba(0.8, 0.7, 0.6, 1.0).into()),
            transform: te,
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

pub fn collision_broad_phase(
    mut commands: Commands,
    start: ResMut<Tree>,
    mut moved: ResMut<MovedAabbs>,
) {
    //start.create_aabb(aabb)
    for moved_aabb in &moved.moved_aabbs {
        start.query_all_leafs(0);
    }

    moved.moved_aabbs.clear();
}

pub fn collision_check(
    mut commands: Commands,
    spheres: Query<&Capsule>,
    mut capsules: Query<(&mut Collider, &mut Transform), Without<DebugSimplex>>,
    debug_cubes: Query<Entity, With<DebugSimplex>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    for cub in debug_cubes.iter() {
        commands.entity(cub).despawn();
    }

    let mut move_: Vec3 = Vec3::default();

    for (convex1, convex2) in capsules
        .iter()
        .step_by(2)
        .zip(capsules.iter().skip(1).step_by(2))
    {
        let manifold = convex_convex_solver(&convex1.0.convex_hull, &convex2.0.convex_hull);
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
        convex1.0.convex_hull.center += move_;
        convex1.1.translation += move_;
    }
}

pub fn move_sphere(
    keyboard_input: Res<Input<KeyCode>>,
    mut spheres: Query<(&mut Collider, &mut Transform)>,
    mut tree: ResMut<Tree>,
    mut moved: ResMut<MovedAabbs>,
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
            col.convex_hull.center += direction;
            //col.lower_center += direction;
            ren.translation += direction;
            x += 1;
            if direction != Vec3::ZERO {
                moved.moved_aabbs.push(col.collider_id);
                tree.move_aabb(col.collider_id, &col.convex_hull.create_aabb());
            }
        }
    }
}
