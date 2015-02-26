#![allow(improper_ctypes)] //TODO do something about this?
#![allow(unstable)]
#![feature(unsafe_destructor)]
#![feature(concat_idents)]
#![feature(box_syntax)]
#![feature(libc)]

extern crate chipmunk_sys;
extern crate "nalgebra" as na;
extern crate libc;

use libc::c_int;

use std::hash;
use std::mem;

pub use bb::BB;
pub use arbiter::{ContactPointSet, Arbiter, MutArbiter};

pub use body::{
    BodyHandle, BodyBase,
    BodyUpcast, BodyUpcastRef, BodyUpcastMut,
    StaticBody, KinematicBody, DynamicBody,
};

pub use constraint::{
    ConstraintHandle, ConstraintBase,
    ConstraintUpcast, ConstraintUpcastRef, ConstraintUpcastMut,
    PinJoint, SlideJoint, PivotJoint, DampedSpring, DampedRotarySpring,
    RotaryLimitJoint, RatchetJoint, GearJoint, SimpleMotor,
};

pub use shape::{
    PointQueryInfo, SegmentQueryInfo, ShapeFilter,
    ShapeHandle, ShapeBase,
    ShapeUpcast, ShapeUpcastRef, ShapeUpcastMut,
    CircleShape, SegmentShape, PolyShape,
};

pub use space::Space;

pub mod ffi;
pub mod bb;
pub mod arbiter;
pub mod body;
pub mod constraint;
pub mod shape;
pub mod space;

#[cfg(test)]
pub mod test;

#[allow(raw_pointer_derive)]
//#[derive(Copy, PartialEq, Eq, Hash)]
pub struct ObjectHandle<P>(*mut P);

unsafe impl<P> Sync for ObjectHandle<P> {}

impl<P> Copy for ObjectHandle<P> {}

impl<P> PartialEq for ObjectHandle<P> {
    fn eq(&self, other: &ObjectHandle<P>) -> bool { self.0 == other.0 } 
}
impl<P> Eq for ObjectHandle<P> {}

impl<P> hash::Hash for ObjectHandle<P> {
    fn hash<H: hash::Hasher>(&self, state: &mut H){
        self.0.hash(state);
    }
}

impl<P> ObjectHandle<P> {
    pub fn wrap(ptr: *mut P) -> Self { ObjectHandle(ptr) }
    pub fn unwrap(self) -> *mut P { self.0 }
}

/// Correspondes to cpFloat
pub type Scalar = f32;

/// Correspondes to cpVect
pub type Pnt2 = na::Pnt2<Scalar>;

/// Correspondes to cpVect
pub type Vec2 = na::Vec2<Scalar>; 

pub type Rot2 = na::Rot2<Scalar>;

/// Correspondes to cpMat2x2
pub type Mat2 = na::Mat2<Scalar>; 

/// Correspondes to cpTransform
pub type Iso2 = na::Iso2<Scalar>;

pub type Group = ffi::cpGroup;
pub const NO_GROUP: Group = 0;

pub type BitMask = ffi::cpBitmask;
pub const ALL_CATEGORIES: BitMask = !0;

pub type CollisionType = ffi::cpCollisionType;
pub const WILDCARD_COLLISION_TYPE: CollisionType = !0;

pub fn min<T: PartialOrd>(v1: T, v2: T) -> T {
    if v1 <= v2 { v1 } else { v2 }
}

pub fn max<T: PartialOrd>(v1: T, v2: T) -> T {
    if v1 >= v2 { v1 } else { v2 }
}

pub fn clamp<T: PartialOrd>(x: T, min: T, max: T) -> T {
    if x < min { min } else if x > max { max } else { x }
}

/// Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c t percent.
pub fn lerp(f1: Scalar, f2: Scalar, t: Scalar) -> Scalar {
    f1 * (1.0 - t) + f2 * t
}

/// Linearly interpolate from @c f1 to @c f2 by no more than @c d.
pub fn lerpconst(f1: Scalar, f2: Scalar, d: Scalar) -> Scalar {
    f1 + clamp(f2 - f1, -d, d)
}

/// Calculate the moment of inertia for a circle.
/// @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
pub fn moment_for_circle(m: Scalar, r1: Scalar, r2: Scalar, offset: Vec2) -> Scalar {
    unsafe { ffi::cpMomentForCircle(m, r1, r2, na::orig::<Pnt2>() + offset) }
}

/// Calculate area of a hollow circle.
/// @c r1 and @c r2 are the inner and outer diameters. A solid circle has an inner diameter of 0.
pub fn area_for_circle(r1: Scalar, r2: Scalar) -> Scalar {
    unsafe { ffi::cpAreaForCircle(r1, r2) }
}

/// Calculate the moment of inertia for a line segment.
/// Beveling radius is not supported.
pub fn moment_for_segment(m: Scalar, a: Pnt2, b: Pnt2, thickness: Scalar) -> Scalar {
    unsafe { ffi::cpMomentForSegment(m, a, b, thickness) }
}

/// Calculate the area of a fattened (capsule shaped) line segment.
pub fn area_for_segment(a: Pnt2, b: Pnt2, radius: Scalar) -> Scalar {
    unsafe { ffi::cpAreaForSegment(a, b, radius) }
}

/// Calculate the moment of inertia for a solid polygon shape assuming it's center of gravity 
/// is at it's centroid. The offset is added to each vertex.
pub fn moment_for_poly(m: Scalar, verts: &[Pnt2], offset: Vec2, radius: Scalar) -> Scalar {
    unsafe { ffi::cpMomentForPoly(
        m, verts.len() as c_int, verts.as_ptr(), na::orig::<Pnt2>() + offset, radius
    )}
}

/// Calculate the signed area of a polygon. A Clockwise winding gives positive area.
/// This is probably backwards from what you expect, but matches Chipmunk's the winding for 
/// poly shapes.
pub fn area_for_poly(verts: &[Pnt2], radius: Scalar) -> Scalar {
    unsafe { ffi::cpAreaForPoly(verts.len() as c_int, verts.as_ptr(), radius) }
}

/// Calculate the natural centroid of a polygon.
pub fn centroid_for_poly(verts: &[Pnt2]) -> Pnt2 {
    unsafe { ffi::cpCentroidForPoly(verts.len() as c_int, verts.as_ptr()) }
}

/// Calculate the moment of inertia for a solid box.
pub fn moment_for_box(m: Scalar, width: Scalar, height: Scalar) -> Scalar {
    unsafe { ffi::cpMomentForBox(m, width, height) }
}

/*
/// Calculate the convex hull of a given set of points. Returns the count of points in the hull.
/// @c result must be a pointer to a @c cpVect array with at least @c count elements.
/// If @c verts == @c result, then @c verts will be reduced inplace.
/// @c first is an optional pointer to an integer to store where the first vertex in the hull
/// came from (i.e. verts[first] == result[0])
/// @c tol is the allowed amount to shrink the hull when simplifying it. A tolerance of 0.0 
/// creates an exact hull.
int cpConvexHull(int count, const cpVect *verts, cpVect *result, int *first, cpFloat tol);
*/

