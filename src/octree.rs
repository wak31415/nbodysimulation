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

    fn add(&self, b: &physics::Body) -> Octree_member;

    fn within_box(&self, p: &Vector3<f32>) -> bool;

    fn fits_in_corner(&self, p: &Vector3<f32>) -> Corner;
}

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

    fn add(&self, b: &physics::Body) -> Octree_member {
        match self {
            Self::Octree_node {
                geometric_center,
                center_of_mass,
                total_mass,
                side_length,
                population,
                tnw,
                tne,
                tsw,
                tse,
                bnw,
                bne,
                bsw,
                bse
            } => {
                if population == &0usize {
                    let corner = self.fits_in_corner(&b.coordinates);
                    match corner {
                        Corner::Tnw => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                tnw: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tne: None,
                                tsw: None,
                                tse: None,
                                bnw: None,
                                bne: None,
                                bsw: None,
                                bse: None
                            }
                        },
                        Corner::Tne => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                tne: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tnw: None,
                                tsw: None,
                                tse: None,
                                bnw: None,
                                bne: None,
                                bsw: None,
                                bse: None
                            }
                        },
                        Corner::Tsw => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                tsw: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tnw: None,
                                tse: None,
                                tne: None,
                                bnw: None,
                                bne: None,
                                bsw: None,
                                bse: None
                            }
                        },
                        Corner::Tse => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                tse: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tnw: None,
                                tsw: None,
                                tne: None,
                                bnw: None,
                                bne: None,
                                bsw: None,
                                bse: None
                            }
                        },
                        Corner::Bnw => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                bnw: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tne: None,
                                tsw: None,
                                tse: None,
                                tnw: None,
                                bne: None,
                                bsw: None,
                                bse: None
                            }
                        },
                        Corner::Bne => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                bne: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tnw: None,
                                tsw: None,
                                tse: None,
                                bnw: None,
                                tne: None,
                                bsw: None,
                                bse: None
                            }
                        },
                        Corner::Bsw => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                bsw: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tnw: None,
                                tse: None,
                                tne: None,
                                bnw: None,
                                bne: None,
                                tsw: None,
                                bse: None
                            }
                        },
                        Corner::Bse => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                bse: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tnw: None,
                                tsw: None,
                                tne: None,
                                bnw: None,
                                bne: None,
                                bsw: None,
                                tse: None
                            }
                        }
                    }
                } else {
                    let corner = self.fits_in_corner(&b.coordinates);
                    match corner {
                        Corner::Tne => {
                            return Octree_node {
                                geometric_center: geometric_center.clone(),
                                center_of_mass: b.coordinates.clone(),
                                total_mass: b.mass.clone(),
                                side_length: side_length.clone(),
                                population: 1usize,
                                tne: Some(Box::new(Octree_leaf {
                                    body: Some(b.clone()),
                                    geometric_center: new_geometric_center(geometric_center, side_length, &corner),
                                    side_length: side_length/&2.0f32
                                })),
                                tnw: None,
                                tsw: None,
                                tse: None,
                                bnw: None,
                                bne: None,
                                bsw: None,
                                bse: None
                            }
                        }
                    }
                }
            }
            Self::Octree_leaf {body, geometric_center, side_length} => {
                assert_eq!(self.within_box(&b.coordinates), true);
                match body {
                    None => { 
                        return Octree_leaf {
                            body: Some(b.clone()),
                            geometric_center: (geometric_center.clone()),
                            side_length: (side_length.clone())
                        } 
                    },
                    Some(o) => {
                        let mut new_node = Octree_node {
                            geometric_center: geometric_center.clone(),
                            center_of_mass: geometric_center.clone(),
                            total_mass: 0.0f32,
                            side_length: side_length/2.0f32,
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

                        new_node = new_node.add(o);
                        new_node = new_node.add(b);

                        return new_node;
                    }
                }
            }
        }
    }

    fn within_box(&self, p: &Vector3<f32>) -> bool {
        let res = |geometric_center: &Vector3<f32>, side_length: &f32| {
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
        // final result
        let res = |gc: &Vector3<f32>, sl: &f32| {
            // if in the top (z)
            if within_top(gc, sl) {
                if within_north(gc, sl) {
                    if within_east(gc, sl) {
                        Corner::Tne
                    } else {
                        Corner::Tnw
                    }
                } else {
                    if within_east(gc, sl) {
                        Corner::Tse
                    } else {
                        Corner::Tsw
                    }
                }
            } else {
                // in the bottom (z)
                if within_north(gc, sl) {
                    if within_east(gc, sl) {
                        Corner::Bne
                    } else {
                        Corner::Bnw
                    }
                } else {
                    if within_east(gc, sl) {
                        Corner::Bse
                    } else {
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
    let by: &f32 = &(sl/&2.0f32);
    return &(c - by) < p && p < &(c + by)
}

fn between_f32(min: &f32, max: &f32, p: &f32) -> bool {
    return min < p && p < max
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

fn new_center_of_mass(x1: &Vector3<f32>, m1: &f32, x2: &Vector3<f32>, m2: &f32)

fn bounding_box_with_center(objects: &Vec<physics::Body>) -> (Vector3<f32>, Vector3<f32>, Vector3<f32>) {
    assert_ne!(objects.len(), 0usize);
    let mut min_x: f32 = objects[0].coordinates[0];
    let mut min_y: f32 = objects[0].coordinates[1];
    let mut min_z: f32 = objects[0].coordinates[2];
    let mut max_x: f32 = objects[0].coordinates[0];
    let mut max_y: f32 = objects[0].coordinates[1];
    let mut max_z: f32 = objects[0].coordinates[2];
    
    for &obj in objects {
        if obj.coordinates[0] < min_x {
            min_x = obj.coordinates[0];
        }
        if obj.coordinates[1] < min_y {
            min_y = obj.coordinates[1];
        }
        if obj.coordinates[2] < min_z {
            min_z = obj.coordinates[2];
        }

        if obj.coordinates[0] < max_x {
            max_x = obj.coordinates[0];
        }
        if obj.coordinates[1] < max_y {
            max_y = obj.coordinates[1];
        }
        if obj.coordinates[2] < max_z {
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
    let x: f32 = (max[0] - min[0]) / 2.0f32;
    let y: f32 = (max[1] - min[1]) / 2.0f32;
    let z: f32 = (max[2] - min[2]) / 2.0f32;
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
    let start_side_len = min_cube_side_len(&start_bsw, &start_tne);

    let mut root_node = Octree_member::Octree_leaf {
        body: None,
        geometric_center: start_center,
        side_length: start_side_len
    };

    // for &obj in objects {
    //     let mut node = 
        
    //     obj.
    // }
}