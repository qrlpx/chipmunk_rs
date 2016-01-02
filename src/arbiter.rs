use ::{Scalar, Pnt2, Vec2};
use shape::ShapeHandle;
use body::BodyHandle;
use ffi;

use libc::c_int;
use std::ops::Deref;
use std::marker::PhantomData;
use std::mem;

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactPoint {
	/// The position of the contact on the surface of the first shape.
    pub point_a: Pnt2,

    /// The position of the contact on the surface of the second shape.
    pub point_b: Pnt2,

	/// Penetration distance of the two shapes. Overlapping means it will be negative.
	/// This value is calculated as cpvdot(cpvsub(point2, point1), normal) and is ignored by 
    /// cpArbiterSetContactPointSet().
    pub distance: Scalar,
}

/// A struct that wraps up the important collision data for an arbiter.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ContactPointSet {
	count: c_int,
	normal: Vec2,
    points: [ContactPoint; ffi::CP_MAX_CONTACTS_PER_ARBITER],
}

impl ContactPointSet {
    pub fn new(normal: Vec2, points: &[ContactPoint]) -> Self {
        assert!(points.len() > 0 && points.len() < ffi::CP_MAX_CONTACTS_PER_ARBITER);
        let mut ret = ContactPointSet{
            count: points.len() as c_int,
            normal: normal,
            points: unsafe { mem::uninitialized() },
        };
        for (idx, point) in points.iter().enumerate() {
            ret.points[idx] = *point;
        }
        ret
    }
    pub fn normal(&self) -> Vec2 { self.normal }

    pub fn points(&self) -> &[ContactPoint] {
        &self.points[..self.count as usize]
    }
}

pub struct Arbiter<'a> {
    ptr: *const ffi::cpArbiter,
    swapped: ffi::cpBool,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Arbiter<'a> {
    #[doc(hidden)]
    pub unsafe fn from_ptr(ptr: *const ffi::cpArbiter, swapped: bool) -> Self {
        Arbiter{ ptr: ptr, swapped: if swapped { 1 } else { 0 }, _phantom: PhantomData }
    }

    #[doc(hidden)]
    pub fn as_ptr(&self) -> *const ffi::cpArbiter { self.ptr }

    /// Returns a Arbiter in which the bodies are swapped.
    pub fn inverse(&self) -> Arbiter {
        Arbiter{ ptr: self.as_ptr(), swapped: !self.swapped, _phantom: PhantomData }
    }

    pub fn shapes(&self) -> (ShapeHandle, ShapeHandle) {
        unsafe {
            let (mut a, mut b) = mem::uninitialized();
            ffi::cpArbiterGetShapes_NOSWAP(self.as_ptr(), &mut a, &mut b, self.swapped);
            (ShapeHandle::new(a), ShapeHandle::new(b)) 
        }
    }

    pub fn bodies(&self) -> (BodyHandle, BodyHandle) {
        unsafe {
            let (mut a, mut b) = mem::zeroed();
            ffi::cpArbiterGetBodies_NOSWAP(self.as_ptr(), &mut a, &mut b, self.swapped);
            (BodyHandle::new(a), BodyHandle::new(b)) 
        }
    }

    pub fn restitution(&self) -> Scalar {
        unsafe { ffi::cpArbiterGetRestitution(self.as_ptr()) }
    }

    pub fn friction(&self) -> Scalar {
        unsafe { ffi::cpArbiterGetFriction(self.as_ptr()) }
    }

    pub fn surface_velocity(&self) -> Vec2 {
        unsafe { ffi::cpArbiterGetSurfaceVelocity_NOSWAP(self.as_ptr() as *mut _, self.swapped) }
    }

    /// Calculate the total impulse including the friction that was applied by this arbiter.
    /// This function should only be called from a post-solve, post-step or cpBodyEachArbiter 
    /// callback.
    pub fn total_impulse(&self) -> Vec2 {
        unsafe { ffi::cpArbiterTotalImpulse_NOSWAP(self.as_ptr(), self.swapped) }
    }

    /// Calculate the amount of energy lost in a collision including static, but not dynamic 
    /// friction. This function should only be called from a post-solve, post-step or 
    /// cpBodyEachArbiter callback.
    pub fn total_ke(&self) -> Scalar {
        unsafe { ffi::cpArbiterTotalKE(self.as_ptr()) }
    }

    /// Return a contact set from an arbiter.
    pub fn contacts(&self) -> ContactPointSet {
        unsafe { mem::transmute(ffi::cpArbiterGetContactPointSet_NOSWAP(self.as_ptr(), self.swapped)) }
    }

    /// Returns true if this is the first step a pair of objects started colliding.
    pub fn is_first_contact(&self) -> bool {
        unsafe { ffi::cpArbiterIsFirstContact(self.as_ptr()) == 1 }
    }

    /// Returns true if in separate callback due to a shape being removed from the space.
    pub fn is_removal(&self) -> bool {
        unsafe { ffi::cpArbiterIsRemoval(self.as_ptr()) == 1 }
    }
}

pub struct ArbiterMut<'a>(Arbiter<'a>);

impl<'a> Deref for ArbiterMut<'a> {
    type Target = Arbiter<'a>;
    fn deref(&self) -> &Arbiter<'a> { &self.0 }
}

impl<'a> ArbiterMut<'a> {
    #[doc(hidden)]
    pub unsafe fn from_mut_ptr(ptr: *mut ffi::cpArbiter, swapped: bool) -> Self {
        ArbiterMut(Arbiter::from_ptr(ptr, swapped))
    }
    
    #[doc(hidden)]
    pub fn as_mut_ptr(&mut self) -> *mut ffi::cpArbiter { self.as_ptr() as *mut _ }

    pub fn inverse_mut(&mut self) -> ArbiterMut {
        ArbiterMut(self.inverse())
    }

    pub fn set_restitution(&mut self, restitution: Scalar){
        unsafe { ffi::cpArbiterSetRestitution(self.as_mut_ptr(), restitution); }
    }
    
    pub fn set_friction(&mut self, friction: Scalar){
        unsafe { ffi::cpArbiterSetFriction(self.as_mut_ptr(), friction); }
    }

    /// Override the relative surface velocity of the two shapes in contact.
    /// By default this is calculated to be the difference of the two
    /// surface velocities clamped to the tangent plane.
    pub fn set_surface_velocity(&mut self, vr: Vec2){
        unsafe { ffi::cpArbiterSetSurfaceVelocity_NOSWAP(self.as_mut_ptr(), vr, self.swapped) }
    }

    /// Replace the contact point set for an arbiter.
    /// This can be a very powerful feature, but use it with caution!
    pub fn set_contacts(&mut self, mut set: ContactPointSet){
        unsafe { ffi::cpArbiterSetContactPointSet_NOSWAP(self.as_mut_ptr(), mem::transmute(&mut set), self.swapped); }
    }

/* 
cpBool cpArbiterCallWildcardBeginA(cpArbiter *arb, cpSpace *space);
cpBool cpArbiterCallWildcardBeginB(cpArbiter *arb, cpSpace *space);

cpBool cpArbiterCallWildcardPreSolveA(cpArbiter *arb, cpSpace *space);
cpBool cpArbiterCallWildcardPreSolveB(cpArbiter *arb, cpSpace *space);

void cpArbiterCallWildcardPostSolveA(cpArbiter *arb, cpSpace *space);
void cpArbiterCallWildcardPostSolveB(cpArbiter *arb, cpSpace *space);

void cpArbiterCallWildcardSeparateA(cpArbiter *arb, cpSpace *space);
void cpArbiterCallWildcardSeparateB(cpArbiter *arb, cpSpace *space);
*/

}

