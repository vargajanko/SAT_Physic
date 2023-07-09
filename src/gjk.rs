use std::ops::Neg;

use bevy::math::*;

use crate::{closet_point_on_segment, ConvexHull, GjkSupport};

//counter clockwise order for the simplex
pub fn gjk(convex: &ConvexHull, point: &impl GjkSupport) -> (bool, Vec3, usize, [Vec3; 4]) {
    //get first support point of the convexhull in
    //direction of the point
    let direction = point.get_support(&Vec3::new(0.0, 0.0, 0.0));
    let x = convex.get_support(&direction) - point.get_support(&-direction);
    //next support point will be searched in the opposite
    //direction of the previous support point
    let mut search_dir = -x;
    let mut simplex: [Vec3; 4] = [Vec3::default(); 4];
    let mut simplex_size = 0;
    simplex[simplex_size] = x;
    simplex_size += 1;
    let mut counter = 0;
    while counter < 10 {
        // println!("how many vertex in mikowski sum are added: {}", counter + 1);
        // println!("simplex position {}", simplex[simplex_size - 1]);
        let enclose_origin: bool;
        let new_found_support_point =
            convex.get_support(&search_dir) + -point.get_support(&-search_dir);
        if new_found_support_point.dot(search_dir) < 0.0 {
            //shapes are not overlapping,
            //now find the closet distance between the two not overlapping shapes
            if check_if_new_support_point_already_added(
                &simplex,
                simplex_size,
                new_found_support_point,
            ) || simplex_size == 3
            {
                if simplex_size == 1 {
                    return (false, simplex[0], simplex_size, simplex);
                } else if simplex_size == 2 {
                    let closet_point_on_line_to_origin = closet_point_on_segment(
                        simplex[simplex_size - 2],
                        simplex[simplex_size - 1],
                        Vec3::new(0.0, 0.0, 0.0),
                    );
                    return (false, closet_point_on_line_to_origin, simplex_size, simplex);
                } else {
                    /*let mut closet_point_on_line_to_origin =
                        closet_point_on_segment(simplex[1], simplex[2], Vec3::new(0.0, 0.0, 0.0));

                    let mut best = closet_point_on_line_to_origin;
                    closet_point_on_line_to_origin =
                        closet_point_on_segment(simplex[0], simplex[2], Vec3::new(0.0, 0.0, 0.0));

                    if closet_point_on_line_to_origin.length_squared() < best.length_squared() {
                        best = closet_point_on_line_to_origin;
                    }
                    closet_point_on_line_to_origin =
                        closet_point_on_segment(simplex[0], simplex[1], Vec3::new(0.0, 0.0, 0.0));

                    if closet_point_on_line_to_origin.length_squared() < best.length_squared() {
                        best = closet_point_on_line_to_origin;
                    }*/
                    let ab = simplex[2] - simplex[1];
                    let ac = simplex[2] - simplex[0];
                    let mut abc = ab.cross(ac);
                    if abc.dot(simplex[2]) < 0.0 {
                        abc = abc.neg();
                    }
                    let plane_distance = abc.dot(simplex[2]);
                    //if plane_distance * plane_distance < best.length_squared() {
                    let t = (Vec3::dot(abc, Vec3::new(0.0, 0.0, 0.0)) - plane_distance)
                        / Vec3::dot(abc, abc);

                    let best = Vec3::new(0.0, 0.0, 0.0) - t * abc;
                    //}
                    return (false, best, simplex_size, simplex);
                }
            }
        }
        simplex[simplex_size] = new_found_support_point;
        simplex_size += 1;

        (enclose_origin, search_dir) = do_simplex(&mut simplex, &mut simplex_size);
        if enclose_origin {
            return (enclose_origin, Vec3::default(), simplex_size, simplex);
        }
        counter += 1;
    }
    (false, Vec3::default(), simplex_size, simplex)
}

fn check_if_new_support_point_already_added(
    simplex: &[Vec3; 4],
    size: usize,
    new_point: Vec3,
) -> bool {
    for x in 0..size {
        if simplex[x] == new_point {
            return true;
        }
    }
    false
}

fn do_simplex(simplex: &mut [Vec3; 4], simplex_size: &mut usize) -> (bool, Vec3) {
    let new_simplex: Vec3;
    let mut enclose_origin: bool = false;
    match simplex_size {
        2 => new_simplex = do_simplex2(simplex, simplex_size),
        3 => new_simplex = do_simplex3(simplex, simplex_size),
        4 => (enclose_origin, new_simplex) = do_simplex4(simplex, simplex_size),
        _ => new_simplex = Vec3::default(),
    }
    (enclose_origin, new_simplex)
}

fn do_simplex2(simplex: &mut [Vec3; 4], simplex_size: &mut usize) -> Vec3 {
    let ab = simplex[0] - simplex[1];
    if ab.dot(-simplex[1]) < 0.0 {
        panic!("do_simplex2: never be here");
        simplex[0] = simplex[1];
        *simplex_size = 1;
        -simplex[1]
    } else {
        ab.cross(-simplex[1]).cross(ab)
    }
}

fn do_simplex3(simplex: &mut [Vec3; 4], simplex_size: &mut usize) -> Vec3 {
    let ab = simplex[1] - simplex[2];
    let ac = simplex[0] - simplex[2];
    let triangle_face = ab.cross(ac);
    //check if triangle face is pointing towards origin
    //if not swap c, b point so it points to it
    if triangle_face.dot(-simplex[2]) < 0.0 {
        simplex.swap(0, 1);
        return -triangle_face;
    }
    triangle_face
}
fn do_simplex4(simplex: &mut [Vec3; 4], simplex_size: &mut usize) -> (bool, Vec3) {
    for x in 0..3 {
        let j = (x + 1) % 3;
        let b = simplex[j];
        let c = simplex[x];
        let triangle_center = simplex[3] + b + c;
        let search_direction = (b - simplex[3]).cross(c - simplex[3]);
        let side_of_origin_to_plane = search_direction.dot(-triangle_center);
        if side_of_origin_to_plane < 0.0 {
            //origin is behind the face
            continue;
        }
        simplex[0] = c;
        simplex[1] = b;
        simplex[2] = simplex[3];
        *simplex_size = 3;
        return (false, search_direction);
    }
    (true, Vec3::default())
}

/* //gjk from casey muratori
fn do_simplex2(simplex: &mut [Vec3; 4], simplex_size: &mut usize) -> Vec3 {
    let ab = simplex[0] - simplex[1];
    if ab.dot(-simplex[1]) > 0.0 {
        ab.cross(-simplex[1]).cross(ab)
    } else {
        println!("do_simplex2: origin is behind A. This should be impossible");
        simplex[0] = simplex[1];
        *simplex_size = 1;
        -simplex[0]
    }
}

fn do_simplex3(simplex: &mut [Vec3; 4], simplex_size: &mut usize) -> Vec3 {
    //direction of newest point to origin
    let a0 = -simplex[2];
    //face of triangle
    let ab = simplex[2] - simplex[1];
    let ac = simplex[2] - simplex[0];
    //face direction of triangle
    let abc = ab.cross(ac);

    //area between a,c and outside of the triangle
    //normal of ac away from triangle
    let ac_normal = abc.cross(ac);

    if ac_normal.dot(a0) > 0.0 {
        //origin is on ac_normal side
        if ac.dot(a0) > 0.0 {
            //origin is between a,c and outside of current triangle
            return ac.cross(a0).cross(ac);
        } else {
            return a0;
        }
    } else {
        let ab_normal = ab.cross(abc);
        if ab_normal.dot(a0) > 0.0 {
            if ab_normal.dot(ab) > 0.0 {
                return ab.cross(a0).cross(ab);
            } else {
                return a0;
            }
        } else {
            //inside the triangle.
            //above or below it?
            if abc.dot(a0) > 0.0 {
                return abc;
            } else {
                return -abc;
            }
        }
    }
}

fn do_simplex4(simplex: &mut [Vec3; 4], simplex_size: &mut usize) -> Vec3 {
    true
}
*/

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn gjk_convex_point() {
        let cube_vertices = vec![
            Vec3::new(-0.5, -0.5, -0.5),
            Vec3::new(-0.5, -0.5, 0.5),
            Vec3::new(-0.5, 0.5, -0.5),
            Vec3::new(-0.5, 0.5, 0.5),
            Vec3::new(0.5, -0.5, -0.5),
            Vec3::new(0.5, -0.5, 0.5),
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(0.5, 0.5, -0.5),
        ];

        let triangle_vertices = vec![
            Vec3::new(-0.5, -0.5, 0.0),
            Vec3::new(-0.5, 0.5, 0.0),
            Vec3::new(0.5, 0.5, 0.0),
        ];

        let convex = ConvexHull {
            center: Vec3::new(0.0, 0.0, 0.0),
            vertices: cube_vertices,
            edges: Default::default(),
            faces: Default::default(),
            planes: Default::default(),
            orientation: todo!(),
        };
        let point = Vec3::new(1.1149588, -0.24032414, -0.15938933);
        let (gjka, what, _, _) = gjk(&convex, &point);
        assert!(gjka);
    }
}
