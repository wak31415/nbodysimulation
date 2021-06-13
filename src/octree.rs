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

pub trait Octree {
    fn center_of_mass(&self) -> Vector3<f32>;
    fn total_mass(&self) -> f32;
    fn side_length(&self) -> f32;
    
    fn population(&self) -> usize;
    // fn level(&self) -> usize;
    fn is_leaf(&self) -> bool;

    fn tnw(&self) -> Option<Box<Octree_member>>;
    fn tne(&self) -> Option<Box<Octree_member>>;
    fn tsw(&self) -> Option<Box<Octree_member>>;
    fn tse(&self) -> Option<Box<Octree_member>>;
    fn bnw(&self) -> Option<Box<Octree_member>>;
    fn bne(&self) -> Option<Box<Octree_member>>;
    fn bsw(&self) -> Option<Box<Octree_member>>;
    fn bse(&self) -> Option<Box<Octree_member>>;

    fn add(&self, b: &physics::Body) -> Octree_member;

    fn within_box(&self, p: &Vector3<f32>) -> bool;
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
    fn center_of_mass(&self) -> Vector3<f32> {
        match self {
            Self::Octree_node => (self.center_of_mass),
            Self::Octree_leaf => {
                match self.body {
                    None => self.geometric_center,
                    Some(b) => b.coordinates
                }
            }
        }
    }

    fn total_mass(&self) -> f32 {
        match self {
			Self::Octree_node => {self.total_mass},
			Self::Octree_leaf => {
                match self.body {
                    None => 0.0f32,
                    Some(b) => b.mass
                }
            }
		}
    }
    fn side_length(&self) -> f32 {
        self.side_length
    }
    
    fn population(&self) -> usize {
        match self {
			Self::Octree_node => {self.population},
			Self::Octree_leaf => {
                match self.body {
                    None => 0usize,
                    Some(_) => 1usize
                }
            }
		}
    }
    // fn level(&self) -> usize
    fn is_leaf(&self) -> bool {
        match self {
			Self::Octree_node => {false},
			Self::Octree_leaf => {true}
		}
    }

    fn tnw(&self) -> Option<Box<Octree_member>> {
        match self {
			Self::Octree_node => {self.tnw},
			Self::Octree_leaf => {None}
		}
    }
    fn tne(&self) -> Option<Box<Octree_member>> {
        match self {
			Self::Octree_node => {self.tne},
			Self::Octree_leaf => {None}
		}
    }
    fn tsw(&self) -> Option<Box<Octree_member>> {
        match self {
			Self::Octree_node => {self.tsw},
			Self::Octree_leaf => {None}
		}
    }
    fn tse(&self) -> Option<Box<Octree_member>> {
        match self {
			Self::Octree_node => {self.tse},
			Self::Octree_leaf => {None}
		}
    }
    fn bnw(&self) -> Option<Box<Octree_member>> {
        match self {
			Self::Octree_node => {self.bnw},
			Self::Octree_leaf => {None}
		}
    }
    fn bne(&self) -> Option<Box<Octree_member>> {
        match self {
			Self::Octree_node => {self.bne},
			Self::Octree_leaf => {None}
		}
    }
    fn bsw(&self) -> Option<Box<Octree_member>> {
        match self {
			Self::Octree_node => {self.bsw},
			Self::Octree_leaf => {None}
		}
    }
    fn bse(&self) -> Option<Box<Octree_member>> {
        match self {
			Self::Octree_node => {self.bse},
			Self::Octree_leaf => {None}
		}
    }

    fn add(&self, b: &physics::Body) -> Octree_member {
        match self {
            Self::Octree_node => {}
            Self::Octree_leaf => {
                assert_eq!(self.within_box(b.coordinates), true);
                match self.body {
                    None => { 
                        return Octree_leaf {
                            body: Some(b.clone()),
                            geometric_center: (self.geometric_center),
                            side_length: (self.side_length)
                        } 
                    },
                    Some => {}
                }
            }
        }
    }

    fn within_box(&self, p: &Vector3<f32>) -> bool {
        match self {
            Self::Octree_node => {}
            Self::Octree_leaf => {
                fn within(c: &f32, sl: &f32, p: &f32) -> bool {
                    let by: &f32 = &(sl/&2.0f32);
                    return &(c - by) < p && p < &(c + by)
                }
                
                return within(self.geometric_center[0], self.side_length, &p[0]) &&
                       within(self.geometric_center[1], self.side_length, &p[1]) &&
                       within(self.geometric_center[2], self.side_length, &p[2])
            }
        }
    }
}

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