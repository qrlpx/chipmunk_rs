use ::{Scalar, Pnt2, Vec2};
use na;
use num::Float;

/// Chipmunk's axis-aligned 2D bounding box type. (left, bottom, right, top)
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct BB {
    pub l: Scalar,
    pub b: Scalar,
    pub r: Scalar,
    pub t: Scalar
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

    pub fn width(&self) -> Scalar { self.r - self.l }
    pub fn height(&self) -> Scalar { self.b - self.t }

    pub fn bottom_left(&self) -> Pnt2 { Pnt2::new(self.l, self.b) }
    pub fn bottom_right(&self) -> Pnt2 { Pnt2::new(self.r, self.b) }
    pub fn top_right(&self) -> Pnt2 { Pnt2::new(self.r, self.t) }
    pub fn top_left(&self) -> Pnt2 { Pnt2::new(self.l, self.t) }

    /// Returns true if @c a and @c b intersect.
    pub fn intersects(&self, b: BB) -> bool {
        self.l <= b.r && b.l <= self.r && self.b <= b.t && b.b <= self.t
    }
    
    /// Returns true if @c other lies completely within @c bb.
    pub fn contains_bb(&self, other: BB) -> bool {
        self.l <= other.l && self.r >= other.r && self.b <= other.b && self.t >= other.t
    }
    
    /// Returns true if @c bb contains @c v.
    pub fn contains_pnt(&self, v: Pnt2) -> bool {
        self.l < v.x && self.r >= v.x && self.b <= v.y && self.t >= v.y
    }

    /// Returns a bounding box that holds both bounding boxes.
    pub fn merge_bb(&self, b: BB) -> BB {
        BB::new(self.l.min(b.l), self.b.min(b.b), self.r.max(b.r), self.t.max(b.t))
    }
    
    /// Returns a bounding box that holds both @c bb and @c v.
    pub fn merge_pnt(&self, v: Pnt2) -> BB {
        BB::new(self.l.min(v.x), self.b.min(v.y), self.r.max(v.x), self.t.max(v.y))
    }
    
    /// Returns the center of a bounding box.
    pub fn center(&self) -> Pnt2 {
        na::center(&Pnt2::new(self.l, self.b), &Pnt2::new(self.r, self.t))
    }

    /// Returns the area of the bounding box.
    pub fn area(&self) -> Scalar {
        (self.r - self.l) * (self.t - self.b)
    }

    /// Merges @c a and @c b and returns the area of the merged bounding box.
    pub fn merged_area(&self, b: BB) -> Scalar {
        self.merge_bb(b).area()
    }

    // TODO any actual need for this?
    /// Returns the fraction along the segment query the cpBB is hit. 
    /// Returns None if it doesn't hit.
    pub fn segment_query(&self, a: Pnt2, b: Pnt2) -> Option<Scalar> {
        let idx = 1.0 / (b.x - a.x);
        let tx1 = if self.l == a.x { Scalar::neg_infinity() } else { (self.l - a.x) * idx };
        let tx2 = if self.r == a.x { Scalar::infinity() } else { (self.r - a.x) * idx };
        let txmin = tx1.min(tx2);
        let txmax = tx1.max(tx2);

        let idy = 1.0 / (b.y - a.y);
        let ty1 = if self.b == a.y { Scalar::neg_infinity() } else { (self.b - a.y) * idy };
        let ty2 = if self.t == a.y { Scalar::infinity() } else { (self.t - a.y) * idy };
        let tymin = ty1.min(ty2);
        let tymax = ty1.max(ty2);

        if tymin <= txmax && txmin <= tymax {
            let rmin = txmin.max(tymin);
            let rmax = txmax.min(tymax);

            if 0.0 <= rmax && rmin <= 1.0 && rmin != Scalar::neg_infinity() {
                return Some(rmin.max(0.0)) 
            }
        }
        None
    }

    // TODO any actual need for this?
    /// Return true if the bounding box intersects the line segment with ends @c a and @c b.
    pub fn intersects_segment(&self, a: Pnt2, b: Pnt2) -> bool {
        match self.segment_query(a, b) {
            Some(_) => true,
            None => false
        }
    }
    
    /// Clamp a vector to a bounding box.
    pub fn clamp_vect(&self, v: Vec2) -> Pnt2 {
        Pnt2::new(na::clamp(v.x, self.l, self.r), na::clamp(v.y, self.b, self.t))
    }
    
    /// Wrap a vector to a bounding box.
    pub fn wrap_vect(&self, v: Vec2) -> Pnt2 {
        let dx = (self.r - self.l).abs();
        let modx = (v.x - self.l) % dx;
        let x = if modx > 0.0 { modx } else { modx + dx };
    
    
        let dy = (self.t - self.b).abs();
        let mody = (v.y - self.b) % dy;
        let y = if mody > 0.0 { mody } else { mody + dy };
    
        Pnt2::new(x + self.l, y + self.b)
    }
    
    /// Returns a bounding box offsetted by @c v.
    pub fn offset(&self, v: Vec2) -> BB {
        BB::new(self.l + v.x, self.b + v.y, self.r + v.x, self.t + v.y)
    }
}

