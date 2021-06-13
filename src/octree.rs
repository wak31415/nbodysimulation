/* Octree:
 *  ____ ____    y
 * | nw | ne |   ^
 * |____|____|   |
 * | sw | se |    --> x
 * |____|____|
 * 
 *  ______ ______    
 * |      |      |   z
 * | tsw  | tse  |   ^
 * |______|______|   |
 * |      |      |    --> x
 * | bsw  | bse  |
 * |______|______|
 * 
 * where the naming convention is:
 * t for top / b for bottom, top is towards +z
 * n for north / s for south, north is towards +y
 * e for east / w for west, east is towards +x
 */

use nalgebra::Vector3;
use super::physics;

#[derive(Debug)]
pub enum Corner {
    Tnw,
    Tne,
    Tsw,
    Tse,
    Bnw,
    Bne,
    Bsw,
    Bse
}

pub trait Octree {
    fn center_of_mass(&self) -> &Vector3<f32>;
    fn total_mass(&self) -> f32;
    fn side_length(&self) -> f32;
    
    fn population(&self) -> usize;
    // fn level(&self) -> usize;
    fn is_leaf(&self) -> bool;

    fn tnw(&self) -> &Option<Box<Octree_member>>;
    fn tne(&self) -> &Option<Box<Octree_member>>;
    fn tsw(&self) -> &Option<Box<Octree_member>>;
    fn tse(&self) -> &Option<Box<Octree_member>>;
    fn bnw(&self) -> &Option<Box<Octree_member>>;
    fn bne(&self) -> &Option<Box<Octree_member>>;
    fn bsw(&self) -> &Option<Box<Octree_member>>;
    fn bse(&self) -> &Option<Box<Octree_member>>;

    fn add(&mut self, b: &physics::Body) -> ();

    fn within_box(&self, p: &Vector3<f32>) -> bool;

    fn fits_in_corner(&self, p: &Vector3<f32>) -> Corner;
}

#[derive(Debug)]
pub enum Octree_member {
    Octree_node {
        geometric_center: Vector3<f32>,
        center_of_mass: Vector3<f32>,
        total_mass: f32,
        side_length: f32,

        population: usize,
        tnw: Option<Box<Octree_member>>,
        tne: Option<Box<Octree_member>>,
        tsw: Option<Box<Octree_member>>,
        tse: Option<Box<Octree_member>>,
        bnw: Option<Box<Octree_member>>,
        bne: Option<Box<Octree_member>>,
        bsw: Option<Box<Octree_member>>,
        bse: Option<Box<Octree_member>>
    },
    Octree_leaf {
        body: Option<physics::Body>,
        geometric_center: Vector3<f32>,
        side_length: f32
    }
}

// making it natural
pub use Octree_member::Octree_node;
pub use Octree_member::Octree_leaf;

impl Octree for Octree_member {
    fn center_of_mass(&self) -> &Vector3<f32> {
        match self {
            Self::Octree_node {center_of_mass, ..} => (center_of_mass),
            Self::Octree_leaf {body, geometric_center, ..} => {
                match body {
                    None => geometric_center,
                    Some(b) => &(b.coordinates)
                }
            }
        }
    }

    fn total_mass(&self) -> f32 {
        match self {
			Self::Octree_node {total_mass, ..} => {total_mass.clone()},
			Self::Octree_leaf {body, ..} => {
                match body {
                    None => 0.0f32,
                    Some(b) => b.mass.clone()
                }
            }
		}
    }
    fn side_length(&self) -> f32 {
        match self {
            Self::Octree_node {side_length, ..} => side_length.clone(),
            Self::Octree_leaf {side_length, ..} => side_length.clone()
        }
    }
    
    fn population(&self) -> usize {
        match self {
			Self::Octree_node {population, ..} => {population.clone()},
			Self::Octree_leaf {body, ..} => {
                match body {
                    None => 0usize,
                    Some(_) => 1usize
                }
            }
		}
    }
    // fn level(&self) -> usize
    fn is_leaf(&self) -> bool {
        match self {
			Self::Octree_node {..} => {false},
			Self::Octree_leaf {..} => {true}
		}
    }

    fn tnw(&self) -> &Option<Box<Octree_member>> {
        match self {
			Self::Octree_node {tnw, ..} => {tnw},
			Self::Octree_leaf {..} => {&None}
		}
    }
    fn tne(&self) -> &Option<Box<Octree_member>> {
        match self {
			Self::Octree_node {tne, ..} => {tne},
			Self::Octree_leaf {..} => {&None}
		}
    }
    fn tsw(&self) -> &Option<Box<Octree_member>> {
        match self {
			Self::Octree_node {tsw, ..} => {tsw},
			Self::Octree_leaf {..} => {&None}
		}
    }
    fn tse(&self) -> &Option<Box<Octree_member>> {
        match self {
			Self::Octree_node {tse, ..} => {tse},
			Self::Octree_leaf {..} => {&None}
		}
    }
    fn bnw(&self) -> &Option<Box<Octree_member>> {
        match self {
			Self::Octree_node {bnw, ..} => {bnw},
			Self::Octree_leaf {..} => {&None}
		}
    }
    fn bne(&self) -> &Option<Box<Octree_member>> {
        match self {
			Self::Octree_node {bne, ..} => {bne},
			Self::Octree_leaf {..} => {&None}
		}
    }
    fn bsw(&self) -> &Option<Box<Octree_member>> {
        match self {
			Self::Octree_node {bsw, ..} => {bsw},
			Self::Octree_leaf {..} => {&None}
		}
    }
    fn bse(&self) -> &Option<Box<Octree_member>> {
        match self {
			Self::Octree_node {bse, ..} => {bse},
			Self::Octree_leaf {..} => {&None}
		}
    }

    fn add(&mut self, b: &physics::Body) -> () {
        // println!("call add on {:?} ; {:?}", self, b);
        assert_eq!(self.within_box(&b.coordinates), true);
        let corner = self.fits_in_corner(&b.coordinates);
        // println!("{:?} fits in {:?}", b, corner);

        match self {
            Self::Octree_node {
                ref geometric_center,
                ref mut center_of_mass,
                ref mut total_mass,
                ref side_length,
                ref mut population,
                ref mut tnw,
                ref mut tne,
                ref mut tsw,
                ref mut tse,
                ref mut bnw,
                ref mut bne,
                ref mut bsw,
                ref mut bse
            } => {
                if population == &0usize {
                    *center_of_mass = b.coordinates.clone();
                    *total_mass = b.mass.clone();
                    *population = 1usize;
                    
                    match corner {
                        Corner::Tnw => {
                            *tnw = Some(Box::new(Octree_leaf {
                                body: Some(b.clone()),
                                geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                side_length: side_length/&2.0f32
                            }))
                        },
                        Corner::Tne => {
                            *tne = Some(Box::new(Octree_leaf {
                                body: Some(b.clone()),
                                geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                side_length: side_length/&2.0f32
                            }))
                        },
                        Corner::Tsw => {
                            *tsw = Some(Box::new(Octree_leaf {
                                body: Some(b.clone()),
                                geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                side_length: side_length/&2.0f32
                            }))
                        },
                        Corner::Tse => {
                            *tse = Some(Box::new(Octree_leaf {
                                body: Some(b.clone()),
                                geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                side_length: side_length/&2.0f32
                            }))
                        },
                        Corner::Bnw => {
                            *bnw = Some(Box::new(Octree_leaf {
                                body: Some(b.clone()),
                                geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                side_length: side_length/&2.0f32
                            }))
                        },
                        Corner::Bne => {
                            *bne = Some(Box::new(Octree_leaf {
                                body: Some(b.clone()),
                                geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                side_length: side_length/&2.0f32
                            }))
                        },
                        Corner::Bsw => {
                            *bsw = Some(Box::new(Octree_leaf {
                                body: Some(b.clone()),
                                geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                side_length: side_length/&2.0f32
                            }))
                        },
                        Corner::Bse => {
                            *bse = Some(Box::new(Octree_leaf {
                                body: Some(b.clone()),
                                geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                side_length: side_length/&2.0f32
                            }))
                        }
                    }
                } else {
                    *center_of_mass = new_center_of_mass(center_of_mass, total_mass, &b.coordinates, &b.mass);
                    *total_mass = *total_mass + b.mass;
                    *population = *population + 1usize;
                    
                    match corner {
                        Corner::Tne => {
                            match tne {
                                None => {
                                    *tne = Some(Box::new(Octree_leaf {
                                        body: Some(b.clone()),
                                        geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                        side_length: side_length/&2.0f32
                                    }))
                                },
                                Some(ref mut boxed_corner) => boxed_corner.add(b)
                            }
                        },
                        Corner::Tnw => {
                            match tnw {
                                None => {
                                    *tnw = Some(Box::new(Octree_leaf {
                                        body: Some(b.clone()),
                                        geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                        side_length: side_length/&2.0f32
                                    }))
                                },
                                Some(ref mut boxed_corner) => boxed_corner.add(b)
                            }
                        },
                        Corner::Tsw => {
                            match tsw {
                                None => {
                                    *tsw = Some(Box::new(Octree_leaf {
                                        body: Some(b.clone()),
                                        geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                        side_length: side_length/&2.0f32
                                    }))
                                },
                                Some(ref mut boxed_corner) => boxed_corner.add(b)
                            }
                        },
                        Corner::Tse => {
                            match tse {
                                None => {
                                    *tse = Some(Box::new(Octree_leaf {
                                        body: Some(b.clone()),
                                        geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                        side_length: side_length/&2.0f32
                                    }))
                                },
                                Some(ref mut boxed_corner) => boxed_corner.add(b)
                            }
                        }
                        Corner::Bne => {
                            match bne {
                                None => {
                                    *bne = Some(Box::new(Octree_leaf {
                                        body: Some(b.clone()),
                                        geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                        side_length: side_length/&2.0f32
                                    }))
                                },
                                Some(ref mut boxed_corner) => boxed_corner.add(b)
                            }
                        },
                        Corner::Bnw => {
                            match bnw {
                                None => {
                                    *bnw = Some(Box::new(Octree_leaf {
                                        body: Some(b.clone()),
                                        geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                        side_length: side_length/&2.0f32
                                    }))
                                },
                                Some(ref mut boxed_corner) => boxed_corner.add(b)
                            }
                        },
                        Corner::Bsw => {
                            match bsw {
                                None => {
                                    *bsw = Some(Box::new(Octree_leaf {
                                        body: Some(b.clone()),
                                        geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                        side_length: side_length/&2.0f32
                                    }))
                                },
                                Some(ref mut boxed_corner) => boxed_corner.add(b)
                            }
                        },
                        Corner::Bse => {
                            match bse {
                                None => {
                                    *bse = Some(Box::new(Octree_leaf {
                                        body: Some(b.clone()),
                                        geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                        side_length: side_length/&2.0f32
                                    }))
                                },
                                Some(ref mut boxed_corner) => boxed_corner.add(b)
                            }
                        }
                    }
                }
            },
            Self::Octree_leaf {body, geometric_center, side_length} => {
                match body {
                    None => { 
                        *self = Octree_leaf {
                            body: Some(b.clone()),
                            geometric_center: (geometric_center.clone()),
                            side_length: (side_length.clone())
                        } 
                        // *self.body = Some(b.clone())
                    },
                    Some(o) => {
                        let o = o.clone();
                        // let body = body.clone();
                        let geometric_center = geometric_center.clone();
                        let side_length = side_length.clone();

                        *self = Octree_node {
                            geometric_center: geometric_center.clone(),
                            center_of_mass: geometric_center.clone(),
                            total_mass: 0.0f32,
                            side_length: side_length,
                            population: 0usize,
                            tnw: None,
                            tne: None,
                            tsw: None,
                            tse: None,
                            bnw: None,
                            bne: None,
                            bsw: None,
                            bse: None,
                        };

                        self.add(&o);
                        self.add(b);
                    }
                }
            }
        }
    }

    fn within_box(&self, p: &Vector3<f32>) -> bool {
        let res = |geometric_center: &Vector3<f32>, side_length: &f32| {
            // println!("individual res:");
            // println!("1: {:?}", within_f32(&geometric_center[0], side_length, &p[0]));
            // println!("1: {:?}", within_f32(&geometric_center[1], side_length, &p[1]));
            // println!("1: {:?}", within_f32(&geometric_center[2], side_length, &p[2]));

            return within_f32(&geometric_center[0], side_length, &p[0]) &&
                   within_f32(&geometric_center[1], side_length, &p[1]) &&
                   within_f32(&geometric_center[2], side_length, &p[2])
        };
        match self {
            Self::Octree_node {geometric_center, side_length, ..} => {
                res(geometric_center, side_length)
            }
            Self::Octree_leaf {geometric_center, side_length, ..} => {            
                res(geometric_center, side_length)
            }
        }
    }

    fn fits_in_corner(&self, p: &Vector3<f32>) -> Corner {
        assert_eq!(self.within_box(p), true);
        let within_half_coord = |geometric_center: &Vector3<f32>,
                                 side_length: &f32,
                                 c: usize| {
            between_f32(&geometric_center[c], 
                        &(geometric_center[c] + (side_length/&2.0f32)),
                        &p[c])
        };
        let within_other_half_coord = |geometric_center: &Vector3<f32>,
                                       side_length: &f32,
                                       c: usize| {
            between_f32(&(geometric_center[c] - (side_length/&2.0f32)), 
                        &geometric_center[c],
                        &p[c])
        };
        // within top (+z)
        let within_top = |gc: &Vector3<f32>, sl: &f32| {
            within_half_coord(gc, sl, 2usize)
        };
        // within north (+y)
        let within_north = |gc: &Vector3<f32>, sl: &f32| {
            within_half_coord(gc, sl, 1usize)
        };
        // within east (+x)
        let within_east = |gc: &Vector3<f32>, sl: &f32| {
            within_half_coord(gc, sl, 0usize)
        };
        // within bottom (-z)
        let within_bottom = |gc: &Vector3<f32>, sl: &f32| {
            within_other_half_coord(gc, sl, 2usize)
        };
        // within south (-y)
        let within_south = |gc: &Vector3<f32>, sl: &f32| {
            within_other_half_coord(gc, sl, 1usize)
        };
        // within west (-x)
        let within_west = |gc: &Vector3<f32>, sl: &f32| {
            within_other_half_coord(gc, sl, 0usize)
        };
        // final result
        let res = |gc: &Vector3<f32>, sl: &f32| {
            // if in the top (z)
            if within_top(gc, sl) {
                if within_north(gc, sl) {
                    if within_east(gc, sl) {
                        Corner::Tne
                    } else {
                        assert_eq!(within_west(gc,sl), true);
                        Corner::Tnw
                    }
                } else {
                    assert_eq!(within_south(gc,sl), true);
                    if within_east(gc, sl) {
                        Corner::Tse
                    } else {
                        assert_eq!(within_west(gc,sl), true);
                        Corner::Tsw
                    }
                }
            } else {
                // in the bottom (z)
                assert_eq!(within_bottom(gc,sl), true);
                if within_north(gc, sl) {
                    if within_east(gc, sl) {
                        Corner::Bne
                    } else {
                        assert_eq!(within_west(gc,sl), true);
                        Corner::Bnw
                    }
                } else {
                    assert_eq!(within_south(gc,sl), true);
                    if within_east(gc, sl) {
                        Corner::Bse
                    } else {
                        assert_eq!(within_west(gc,sl), true);
                        Corner::Bsw
                    }
                }
            }
        };        
        match self {
            Self::Octree_node {geometric_center, side_length, ..} => {
                res(geometric_center, side_length)
            },
            Self::Octree_leaf {geometric_center, side_length, ..} => {
                res(geometric_center, side_length)
            }
        }
    }
}

fn within_f32(c: &f32, sl: &f32, p: &f32) -> bool {
    // println!("within_f32_a: {:?} {:?} {:?}", c, sl, p);
    let by: &f32 = &(sl/&2.0f32);
    // println!("within_f32_b: {:?} < {:?} <= {:?}", &(c - by), p, &(c + by));
    // println!("within_f32_c: {:?} {:?} {:?}", by, &(c - by) < p, p <= &(c + by));
    return &(c - by) < p && p <= &(c + by)
}

fn between_f32(min: &f32, max: &f32, p: &f32) -> bool {
    assert_eq!(min < max, true);
    // println!("between_f32_a: {:?} < {:?} <= {:?}", min, p, max);
    return min < p && p <= max
}

fn new_geometric_center(ogc: &Vector3<f32>, 
                        side_length: &f32,
                        move_corner: &Corner) -> Vector3<f32> {
    let adder = |v: f32| (v + &(side_length/&4.0f32));
    let remover = |v: f32| (v - &(side_length/&4.0f32));

    match move_corner {
        Corner::Tne => {
            Vector3::new(
                // east (+x)
                adder(ogc[0]),
                // north (+y)
                adder(ogc[1]),
                // top (+z)
                adder(ogc[2])
            )
        },
        Corner::Tnw => {
            Vector3::new(
                // west (-x)
                remover(ogc[0]),
                // north (+y)
                adder(ogc[1]),
                // top (+z)
                adder(ogc[2])
            )
        },
        Corner::Tse => {
            Vector3::new(
                // east (+x)
                adder(ogc[0]),
                // south (-y)
                remover(ogc[1]),
                // top (+z)
                adder(ogc[2])
            )
        },
        Corner::Tsw => {
            Vector3::new(
                // west (-x)
                remover(ogc[0]),
                // south (-y)
                remover(ogc[1]),
                // top (+z)
                adder(ogc[2])
            )
        },
        Corner::Bne => {
            Vector3::new(
                // east (+x)
                adder(ogc[0]),
                // north (+y)
                adder(ogc[1]),
                // bottom (-z)
                remover(ogc[2])
            )
        },
        Corner::Bnw => {
            Vector3::new(
                // west (-x)
                remover(ogc[0]),
                // north (+y)
                adder(ogc[1]),
                // bottom (-z)
                remover(ogc[2])
            )
        },
        Corner::Bse => {
            Vector3::new(
                // east (+x)
                adder(ogc[0]),
                // south (-y)
                remover(ogc[1]),
                // bottom (-z)
                remover(ogc[2])
            )
        },
        Corner::Bsw => {
            Vector3::new(
                // west (-x)
                remover(ogc[0]),
                // south (-y)
                remover(ogc[1]),
                // bottom (-z)
                remover(ogc[2])
            )
        }
    }
}

fn new_center_of_mass(x1: &Vector3<f32>, m1: &f32, x2: &Vector3<f32>, m2: &f32) -> Vector3<f32> {
    let m = m1 + m2;
    let op = |c: usize| (x1[c] * m1 + x2[c] * m2) / m;
    Vector3::new(
        op(0usize),
        op(1usize),
        op(2usize)
    )
}

fn bounding_box_with_center(objects: &Vec<physics::Body>) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    assert_ne!(objects.len(), 0usize);
    let mut min_x: f32 = objects[0].coordinates[0];
    let mut min_y: f32 = objects[0].coordinates[1];
    let mut min_z: f32 = objects[0].coordinates[2];
    let mut max_x: f32 = objects[0].coordinates[0];
    let mut max_y: f32 = objects[0].coordinates[1];
    let mut max_z: f32 = objects[0].coordinates[2];
    
    for ref obj in objects {
        // println!("Helper {:?}", obj);
        if obj.coordinates[0] < min_x {
            min_x = obj.coordinates[0];
        }
        if obj.coordinates[1] < min_y {
            min_y = obj.coordinates[1];
        }
        if obj.coordinates[2] < min_z {
            min_z = obj.coordinates[2];
        }

        if obj.coordinates[0] > max_x {
            max_x = obj.coordinates[0];
        }
        if obj.coordinates[1] > max_y {
            max_y = obj.coordinates[1];
        }
        if obj.coordinates[2] > max_z {
            max_z = obj.coordinates[2];
        }
    }

    let bsw = Vector3::new(min_x, min_y, min_z);
    let tne = Vector3::new(max_x, max_y, max_z);

    return (
        bsw,
        tne,
        bounding_box_center(&bsw, &tne)
    );
}

fn bounding_box_center(min: &Vector3<f32>, max: &Vector3<f32>) -> Vector3<f32> {
    let x: f32 = min[0] + (max[0] - min[0]) / 2.0f32;
    let y: f32 = min[1] + (max[1] - min[1]) / 2.0f32;
    let z: f32 = min[2] + (max[2] - min[2]) / 2.0f32;
    // println!("x {:?} x {:?} x {:?}", x, y, z);
    // println!("max[0] {:?} min[0] {:?} max-min {:?} /2 {:?}", max[0], min[0], max[0] - min[0], x);
    // println!("x {:?} x {:?} x {:?}", x, y, z);
    Vector3::new(x,y,z)
}

fn min_cube_side_len(min: &Vector3<f32>, max_p: &Vector3<f32>) -> f32 {
    let x_dist: f32 = max_p[0] - min[0];
    let y_dist: f32 = max_p[1] - min[1];
    let z_dist: f32 = max_p[2] - min[2];
    if x_dist >= y_dist {
        if x_dist >= z_dist {
            x_dist
        } else {
            z_dist
        } 
    } else {
        if y_dist >= z_dist {
            y_dist
        } else {
            z_dist
        }
    } 
}


pub fn create_octree_sequential(
    objects: &Vec<physics::Body>
) -> Box<Octree_member> {
    let (start_bsw, start_tne, start_center) = bounding_box_with_center(objects);
    // println!("Helper says {:?} {:?} {:?}", start_bsw, start_tne, start_center);
    let start_side_len = min_cube_side_len(&start_bsw, &start_tne) + 0.2f32;
    let center = Vector3::new(
        start_bsw[0] - 0.1f32 + start_side_len / 2.0f32,
        start_bsw[1] - 0.1f32 + start_side_len / 2.0f32,
        start_bsw[2] - 0.1f32 + start_side_len / 2.0f32,
    );

    let mut root_node = Box::new(Octree_member::Octree_leaf {
        body: None,
        geometric_center: center,
        side_length: start_side_len
    });

    // println!("Starting at basic tree {:?}", root_node);
    
    for ref obj in objects {
        // println!("Adding {:?}", obj);
        root_node.add(&obj);
        // println!("New tree: {:?}", root_node);
    }

    return root_node;
}