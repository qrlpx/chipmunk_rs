#![feature(box_syntax)]
#![feature(libc)]
#![feature(nonzero)]
#![feature(hashmap_hasher)]
#![feature(raw)]

extern crate nalgebra as na;
extern crate fnv;
extern crate num;
extern crate libc;
extern crate core;

#[allow(dead_code)]
mod ffi;
mod bb;
#[macro_use]
mod object;
mod shape;
pub mod body;
mod arbiter;
mod constraint;
pub mod space;

pub use bb::BB;
pub use shape::*;
pub use body::{Body, BodyHandle, BodyDowncast, BodyDowncastRef, BodyDowncastMut};
pub use arbiter::*;
pub use constraint::*;
pub use space::{LockedSpace, Space};

// ++++++++++++++++++++ aliases ++++++++++++++++++++

/// Correspondes to cpFloat
#[cfg(not(use_doubles))]
pub type Scalar = f32;

/// Correspondes to cpFloat
#[cfg(use_doubles)]
pub type Scalar = f64;

/// Correspondes to cpVect
pub type Pnt2 = na::Pnt2<Scalar>;

/// Correspondes to cpVect
pub type Vec2 = na::Vec2<Scalar>; 

//TODO remove?
/// Correspondes to cpMat2x2
pub type Rot2 = na::Rot2<Scalar>;

//TODO remove?
/// Correspondes to cpMat2x2
pub type Mat2 = na::Mat2<Scalar>; 

//TODO remove?
/// Correspondes to cpTransform
pub type Iso2 = na::Iso2<Scalar>;

pub type Group = ffi::cpGroup;
pub const NO_GROUP: Group = 0;

pub type Bitmask = ffi::cpBitmask;
pub const ALL_CATEGORIES: Bitmask = !0;

pub type CollisionType = ffi::cpCollisionType;
pub const WILDCARD_COLLISION_TYPE: CollisionType = !0;

// ++++++++++++++++++++ free fns ++++++++++++++++++++

use libc::c_int;
use std::mem;

/// Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c t percent.
pub fn lerp(f1: Scalar, f2: Scalar, t: Scalar) -> Scalar {
    f1 * (1.0 - t) + f2 * t
}

/// Linearly interpolate from @c f1 to @c f2 by no more than @c d.
pub fn lerpconst(f1: Scalar, f2: Scalar, d: Scalar) -> Scalar {
    f1 + na::clamp(f2 - f1, -d, d)
}

/// Calculate the moment of inertia for a circle.
/// @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
pub fn moment_for_circle(m: Scalar, r1: Scalar, r2: Scalar, offset: Vec2) -> Scalar {
    unsafe { ffi::cpMomentForCircle(m, r1, r2, offset) }
}

/// Calculate area of a hollow circle.
/// @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
pub fn area_for_circle(r1: Scalar, r2: Scalar) -> Scalar {
    unsafe { ffi::cpAreaForCircle(r1, r2) }
}

/// Calculate the moment of inertia for a line segment.
/// Beveling radius is not supported.
pub fn moment_for_segment(m: Scalar, a: Pnt2, b: Pnt2, thickness: Scalar) -> Scalar {
    unsafe { ffi::cpMomentForSegment(m, a.to_vec(), b.to_vec(), thickness) }
}

/// Calculate the area of a fattened (capsule shaped) line segment.
pub fn area_for_segment(a: Pnt2, b: Pnt2, radius: Scalar) -> Scalar {
    unsafe { ffi::cpAreaForSegment(a.to_vec(), b.to_vec(), radius) }
}

/// Calculate the moment of inertia for a solid polygon shape assuming it's center of gravity 
/// is at it's centroid. The offset is added to each vertex.
pub fn moment_for_poly(m: Scalar, verts: &[Pnt2], offset: Vec2, radius: Scalar) -> Scalar {
    unsafe { 
        let verts = mem::transmute::<&[Pnt2], &[Vec2]>(verts);
        ffi::cpMomentForPoly(m, verts.len() as c_int, verts.as_ptr(), offset, radius)
    }
}

/// Calculate the signed area of a polygon. A Clockwise winding gives positive area.
/// This is probably backwards from what you expect, but matches Chipmunk's the winding for 
/// poly shapes.
pub fn area_for_poly(verts: &[Pnt2], radius: Scalar) -> Scalar {
    unsafe { 
        let verts = mem::transmute::<&[Pnt2], &[Vec2]>(verts);
        ffi::cpAreaForPoly(verts.len() as c_int, verts.as_ptr(), radius) 
    }
}

/// Calculate the natural centroid of a polygon.
pub fn centroid_for_poly(verts: &[Pnt2]) -> Pnt2 {
    unsafe { 
        let verts = mem::transmute::<&[Pnt2], &[Vec2]>(verts);
        ffi::cpCentroidForPoly(verts.len() as c_int, verts.as_ptr()).to_pnt() 
    }
}

/// Calculate the moment of inertia for a solid box.
pub fn moment_for_box(m: Scalar, width: Scalar, height: Scalar) -> Scalar {
    unsafe { ffi::cpMomentForBox(m, width, height) }
}

/* TODO
/// Calculate the convex hull of a given set of points. Returns the count of points in the hull.
/// @c result must be a pointer to a @c cpVect array with at least @c count elements.
/// If @c verts == @c result, then @c verts will be reduced inplace.
/// @c first is an optional pointer to an integer to store where the first vertex in the hull
/// came from (i.e. verts[first] == result[0])
/// @c tol is the allowed amount to shrink the hull when simplifying it. A tolerance of 0.0 
/// creates an exact hull.
int cpConvexHull(int count, const cpVect *verts, cpVect *result, int *first, cpFloat tol);
*/
/* TODO
/// Returns the closest point on the line segment ab, to the point p.
static inline cpVect
cpClosetPointOnSegment(const cpVect p, const cpVect a, const cpVect b)
{
	cpVect delta = cpvsub(a, b);
	cpFloat t = cpfclamp01(cpvdot(delta, cpvsub(p, b))/cpvlengthsq(delta));
	return cpvadd(b, cpvmult(delta, t));
}
*/
