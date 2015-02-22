use ::{min, max, clamp};
use ::{Scalar, Pnt2, Vec2};

use na;

use std::num::Float;

/// Chipmunk's axis-aligned 2D bounding box type. (left, bottom, right, top)
#[derive(Copy)]
pub struct BB {
    l: Scalar, b: Scalar, r: Scalar, t: Scalar
}

impl BB {
    /// Convenience constructor for cpBB structs.
    pub fn new(l: Scalar, b: Scalar, r: Scalar, t: Scalar) -> Self {
        BB{ l: l, b: b, r: r, t: t }
    }

    /// Constructs a cpBB centered on a point with the given extents (half sizes).
    pub fn for_extents(c: Pnt2, hw: Scalar, hh: Scalar) -> BB {
        BB::new(c.x - hw, c.y - hh, c.x + hw, c.y + hh)
    }

    /// Constructs a cpBB for a circle with the given position and radius.
    pub fn for_circle(p: Pnt2, r: Scalar) -> BB {
        BB::for_extents(p, r, r)
    }

    /// Returns true if @c a and @c b intersect.
    pub fn intersects(self, b: BB) -> bool {
        self.l <= b.r && b.l <= self.r && self.b <= b.t && b.b <= self.t
    }
    
    /// Returns true if @c other lies completely within @c bb.
    pub fn contains_bb(self, other: BB) -> bool {
        self.l <= other.l && self.r >= other.r && self.b <= other.b && self.t >= other.t
    }
    
    /// Returns true if @c bb contains @c v.
    pub fn contains_vect(self, v: Pnt2) -> bool {
        self.l < v.x && self.r >= v.x && self.b <= v.y && self.t >= v.y
    }

    /// Returns a bounding box that holds both bounding boxes.
    pub fn merge_bb(self, b: BB) -> BB {
        BB::new(min(self.l, b.l), min(self.b, b.b), max(self.r, b.r), max(self.t, b.t))
    }
    
    /// Returns a bounding box that holds both @c bb and @c v.
    pub fn merge_vect(self, v: Pnt2) -> BB {
        BB::new(min(self.l, v.x), min(self.b, v.y), max(self.r, v.x), max(self.t, v.y))
    }
    
    /// Returns the center of a bounding box.
    pub fn center(self) -> Pnt2 {
        na::center(&na::Pnt2::new(self.l, self.b), &na::Pnt2::new(self.r, self.t))
    }

    /// Returns the area of the bounding box.
    pub fn area(self) -> Scalar {
        (self.r - self.l) * (self.t - self.b)
    }

    /// Merges @c a and @c b and returns the area of the merged bounding box.
    pub fn merged_area(self, b: BB) -> Scalar {
        self.merge_bb(b).area()
    }

    /// Returns the fraction along the segment query the cpBB is hit. 
    /// Returns None if it doesn't hit.
    pub fn segment_query(self, a: Pnt2, b: Pnt2) -> Option<Scalar> {
        let idx = 1.0 / (b.x - a.x);
        let tx1 = if self.l == a.x { Float::neg_infinity() } else { (self.l - a.x) * idx };
        let tx2 = if self.r == a.x { Float::infinity() } else { (self.r - a.x) * idx };
        let txmin = min(tx1, tx2);
        let txmax = max(tx1, tx2);

        let idy = 1.0 / (b.y - a.y);
        let ty1 = if self.b == a.y { Float::neg_infinity() } else { (self.b - a.y) * idy };
        let ty2 = if self.t == a.y { Float::infinity() } else { (self.t - a.y) * idy };
        let tymin = min(ty1, ty2);
        let tymax = max(ty1, ty2);

        if tymin <= txmax && txmin <= tymax {
            let rmin = max(txmin, tymin);
            let rmax = min(txmax, tymax);

            if 0.0 <= rmax && rmin <= 1.0 {
                return Some(max(rmin, 0.0))
            }
        }
        None
    }

    /// Return true if the bounding box intersects the line segment with ends @c a and @c b.
    pub fn intersects_segment(self, a: Pnt2, b: Pnt2) -> bool {
        match self.segment_query(a, b) {
            Some(_) => true,
            None => false
        }
    }
    
    /// Clamp a vector to a bounding box.
    pub fn clamp_vect(self, v: Pnt2) -> Pnt2 {
        na::Pnt2::new(clamp(v.x, self.l, self.r), clamp(v.y, self.b, self.t))
    }
    
    /// Wrap a vector to a bounding box.
    pub fn wrap_vect(self, v: Pnt2) -> Pnt2 {
        let dx = (self.r - self.l).abs();
        let modx = (v.x - self.l) % dx;
        let x = if modx > 0.0 { modx } else { modx + dx };
    
    
        let dy = (self.t - self.b).abs();
        let mody = (v.y - self.b) % dy;
        let y = if mody > 0.0 { mody } else { mody + dy };
    
        na::Pnt2::new(x + self.l, y + self.b)
    }
    
    /// Returns a bounding box offsetted by @c v.
    pub fn offset(self, v: Vec2) -> BB {
        BB::new(self.l + v.x, self.b + v.y, self.r + v.x, self.t + v.y)
    }
}

