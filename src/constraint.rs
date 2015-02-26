use ::{Scalar, Pnt2, ObjectHandle};
use bb::BB;
use body::{BodyHandle, BodyBase};
use ffi;

use std::ops::{Deref, DerefMut};
use std::{mem, ptr};

pub enum ConstraintUpcast {
    Pin(Box<PinJoint>),
    Slide(Box<SlideJoint>),
    Pivot(Box<PivotJoint>),
    Groove(Box<GrooveJoint>),
    DampedSpring(Box<DampedSpring>),
    DampedRotarySpring(Box<DampedRotarySpring>),
    RotaryLimit(Box<RotaryLimitJoint>),
    Ratchet(Box<RatchetJoint>),
    Gear(Box<GearJoint>),
    SimpleMotor(Box<SimpleMotor>),
}

pub enum ConstraintUpcastRef<'a> {
    Pin(&'a PinJoint),
    Slide(&'a SlideJoint),
    Pivot(&'a PivotJoint),
    Groove(&'a GrooveJoint),
    DampedSpring(&'a DampedSpring),
    DampedRotarySpring(&'a DampedRotarySpring),
    RotaryLimit(&'a RotaryLimitJoint),
    Ratchet(&'a RatchetJoint),
    Gear(&'a GearJoint),
    SimpleMotor(&'a SimpleMotor),
}

pub enum ConstraintUpcastMut<'a> {
    Pin(&'a mut PinJoint),
    Slide(&'a mut SlideJoint),
    Pivot(&'a mut PivotJoint),
    Groove(&'a mut GrooveJoint),
    DampedSpring(&'a mut DampedSpring),
    DampedRotarySpring(&'a mut DampedRotarySpring),
    RotaryLimit(&'a mut RotaryLimitJoint),
    Ratchet(&'a mut RatchetJoint),
    Gear(&'a mut GearJoint),
    SimpleMotor(&'a mut SimpleMotor),
}

pub type ConstraintHandle = ObjectHandle<ffi::cpConstraint>;

pub struct ConstraintBase(ffi::cpConstraint);

unsafe impl Sync for ConstraintBase {}

impl ConstraintBase {
    pub unsafe fn from_ptr(ptr: *const ffi::cpConstraint) -> &'static ConstraintBase {
        mem::transmute(ptr)
    }
    pub unsafe fn from_mut_ptr(ptr: *mut ffi::cpConstraint) -> &'static mut ConstraintBase {
        mem::transmute(ptr)
    }
    pub unsafe fn box_from_ptr(ptr: *const ffi::cpConstraint) -> Box<ConstraintBase> {
        mem::transmute(ptr)
    }

    pub fn upcast(_self: Box<Self>) -> ConstraintUpcast {
        unsafe {
            let ptr = _self.ptr();
            if ffi::cpConstraintIsPinJoint(ptr) == 1 {
                ConstraintUpcast::Pin(mem::transmute(_self))
            } else if ffi::cpConstraintIsSlideJoint(ptr) == 1{
                ConstraintUpcast::Slide(mem::transmute(_self))
            } else if ffi::cpConstraintIsPivotJoint(ptr) == 1{
                ConstraintUpcast::Pivot(mem::transmute(_self))
            } else if ffi::cpConstraintIsGrooveJoint(ptr) == 1 {
                ConstraintUpcast::Groove(mem::transmute(_self))
            } else if ffi::cpConstraintIsDampedSpring(ptr) == 1 {
                ConstraintUpcast::DampedSpring(mem::transmute(_self))
            } else if ffi::cpConstraintIsDampedRotarySpring(ptr) == 1 {
                ConstraintUpcast::DampedRotarySpring(mem::transmute(_self))
            } else if ffi::cpConstraintIsRatchetJoint(ptr) == 1 {
                ConstraintUpcast::Ratchet(mem::transmute(_self))
            } else if ffi::cpConstraintIsGearJoint(ptr) == 1 {
                ConstraintUpcast::Gear(mem::transmute(_self))
            } else if ffi::cpConstraintIsSimpleMotor(ptr) == 1 {
                ConstraintUpcast::SimpleMotor(mem::transmute(_self))
            } else {
                unreachable!()
            }
        }
    }
    pub fn upcast_ref(&self) -> ConstraintUpcastRef {
        unsafe { mem::transmute(ConstraintBase::upcast(mem::transmute(self))) }
    }
    pub fn upcast_mut(&mut self) -> ConstraintUpcastMut {
        unsafe { mem::transmute(ConstraintBase::upcast(mem::transmute(self))) }
    }

    pub fn ptr(&self) -> *const ffi::cpConstraint { &self.0 }
    pub fn mut_ptr(&mut self) -> *mut ffi::cpConstraint { &mut self.0 }
    pub fn handle(&self) -> ConstraintHandle { 
        ObjectHandle::wrap(self.ptr() as *mut _)
    }

    pub unsafe fn set_bodies(&mut self, bodies: Option<(BodyHandle, BodyHandle)>){
        match bodies {
            Some((a, b)) => {
                (*self.mut_ptr()).a = a.unwrap();
                (*self.mut_ptr()).b = b.unwrap();
            }
            None => {
                (*self.mut_ptr()).a = ptr::null_mut();
                (*self.mut_ptr()).b = ptr::null_mut();
            }
        }
    }

    pub fn is_attached(&self) -> bool {
        unsafe { 
            (*self.ptr()).a.is_null() == false 
            && (*self.ptr()).b.is_null() == false
        }
    }

    pub fn body_a(&self) -> &BodyBase {
        debug_assert!(self.is_attached());
        unsafe { BodyBase::from_ptr(ffi::cpConstraintGetBodyA(self.ptr() as *mut _)) }
    }
    pub fn body_b(&self) -> &BodyBase {
        debug_assert!(self.is_attached());
        unsafe { BodyBase::from_ptr(ffi::cpConstraintGetBodyB(self.ptr() as *mut _)) }
    }
    pub fn mut_body_a(&mut self) -> &mut BodyBase {
        unsafe { mem::transmute(self.body_a()) }
    }
    pub fn mut_body_b(&mut self) -> &mut BodyBase {
        unsafe { mem::transmute(self.body_a()) }
    }

    /// Get the maximum force that this constraint is allowed to use.
    pub fn max_force(&self) -> Scalar {
        unsafe { ffi::cpConstraintGetMaxForce(self.ptr()) }
    }
    /// Set the maximum force that this constraint is allowed to use. (defaults to INFINITY)
    pub fn set_max_force(&mut self, max_force: Scalar){
        unsafe { ffi::cpConstraintSetMaxForce(self.mut_ptr(), max_force); }
    }

    /// Get rate at which joint error is corrected.
    pub fn error_bias(&self) -> Scalar {
        unsafe { ffi::cpConstraintGetErrorBias(self.ptr()) }
    }
    /// Set rate at which joint error is corrected.
    /// Defaults to pow(1.0 - 0.1, 60.0) meaning that it will
    /// correct 10% of the error every 1/60th of a second.
    pub fn set_error_bias(&mut self, error_bias: Scalar){
        unsafe { ffi::cpConstraintSetErrorBias(self.mut_ptr(), error_bias); }
    }

    /// Get the maximum rate at which joint error is corrected.
    pub fn max_bias(&self) -> Scalar {
        unsafe { ffi::cpConstraintGetMaxBias(self.ptr()) }
    }
    /// Set the maximum rate at which joint error is corrected. (defaults to INFINITY)
    pub fn set_max_bias(&mut self, max_bias: Scalar){
        unsafe { ffi::cpConstraintSetMaxBias(self.mut_ptr(), max_bias); }
    }

    /// Get if the two bodies connected by the constraint are allowed to collide or not.
    pub fn collide_bodies(&self) -> bool {
        unsafe { ffi::cpConstraintGetCollideBodies(self.ptr()) == 1 }
    }
    /// Set if the two bodies connected by the constraint are allowed to collide or not. 
    /// (defaults to cpFalse)
    pub fn set_collide_bodies(&mut self, collide_bodies: bool) {
        let c = if collide_bodies { 1 } else { 0 };
        unsafe { ffi::cpConstraintSetCollideBodies(self.mut_ptr(), c); }
    }

/*
/// Callback function type that gets called before solving a joint.
typedef void (*cpConstraintPreSolveFunc)(&mut self, cpSpace *space);
/// Callback function type that gets called after solving a joint.
typedef void (*cpConstraintPostSolveFunc)(&mut self, cpSpace *space);

/// Get the pre-solve function that is called before the solver runs.
cpConstraintPreSolveFunc cpConstraintGetPreSolveFunc(&self);
cpConstraintPreSolveFunc cpConstraintGetPreSolveFunc(&self);
/// Set the pre-solve function that is called before the solver runs.
void cpConstraintSetPreSolveFunc(&mut self, cpConstraintPreSolveFunc preSolveFunc);
void cpConstraintSetPreSolveFunc(&mut self, cpConstraintPreSolveFunc preSolveFunc);

/// Get the post-solve function that is called before the solver runs.
cpConstraintPostSolveFunc cpConstraintGetPostSolveFunc(&self);
cpConstraintPostSolveFunc cpConstraintGetPostSolveFunc(&self);
/// Set the post-solve function that is called before the solver runs.
void cpConstraintSetPostSolveFunc(&mut self, cpConstraintPostSolveFunc postSolveFunc);
void cpConstraintSetPostSolveFunc(&mut self, cpConstraintPostSolveFunc postSolveFunc);
*/

    /// Get the last impulse applied by this constraint.
    pub fn last_impulse(&self) -> Scalar {
        unsafe { ffi::cpConstraintGetImpulse(self.ptr() as *mut _) } // TODO: fix const
    }

}

impl Drop for ConstraintBase {
    fn drop(&mut self){
        unsafe { ffi::cpConstraintDestroy(self.mut_ptr()); }
    }
}

macro_rules! impl_constraint {
    ($ty:ty) => (
        impl $ty {
            fn downcast(_self: Box<Self>) -> Box<BodyBase> {
                unsafe { mem::transmute(_self) }
            }
        }
        unsafe impl Sync for $ty {}
        impl Deref for $ty {
            type Target = ConstraintBase;
            fn deref(&self) -> &ConstraintBase { 
                unsafe { mem::transmute(self) }
            }
        }
        impl DerefMut for $ty {
            fn deref_mut(&mut self) -> &mut ConstraintBase {
                unsafe { mem::transmute(self) }
            }
        }
        impl Drop for $ty {
            fn drop(&mut self){
                unsafe { ffi::cpConstraintDestroy(self.mut_ptr()); }
            }
        }
    );
}


pub struct PinJoint(ffi::cpPinJoint);
impl_constraint!(PinJoint);

impl PinJoint {
    pub fn new(anchor_a: Pnt2, anchor_b: Pnt2) -> Self {
        PinJoint(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpPinJointInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), anchor_a, anchor_b
            );
            data
        })
    }

    /// Get the location of the first anchor relative to the first body.
    pub fn anchor_a(&self) -> Pnt2 {
        unsafe { ffi::cpPinJointGetAnchorA(self.ptr()) }
    }
    /// Set the location of the first anchor relative to the first body.
    pub fn set_anchor_a(&mut self, a: Pnt2){
        unsafe { ffi::cpPinJointSetAnchorA(self.mut_ptr(), a); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpPinJointGetAnchorB(self.ptr()) }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpPinJointSetAnchorB(self.mut_ptr(), b); }
    }
    
    /// Get the distance the joint will maintain between the two anchors.
    pub fn dist(&self) -> Scalar {
        unsafe { ffi::cpPinJointGetDist(self.ptr()) }
    }
    /// Set the distance the joint will maintain between the two anchors.
    pub fn set_dist(&mut self, dist: Scalar){
        unsafe { ffi::cpPinJointSetDist(self.mut_ptr(), dist); }
    }
}

pub struct SlideJoint(ffi::cpSlideJoint);
impl_constraint!(SlideJoint);

impl SlideJoint {
    pub fn new(anchor_a: Pnt2, anchor_b: Pnt2, min: Scalar, max: Scalar) -> Self {
        SlideJoint(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpSlideJointInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), anchor_a, anchor_b, min, max
            );
            data
        })
    }

    /// Get the location of the first anchor relative to the first body.
    pub fn anchor_a(&self) -> Pnt2 {
        unsafe { ffi::cpSlideJointGetAnchorA(self.ptr()) }
    }
    /// Set the location of the first anchor relative to the first body.
    pub fn set_anchor_a(&mut self, a: Pnt2){
        unsafe { ffi::cpSlideJointSetAnchorA(self.mut_ptr(), a); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpSlideJointGetAnchorB(self.ptr()) }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpSlideJointSetAnchorB(self.mut_ptr(), b); }
    }
    
    /// Get the distance the joint will maintain between the two anchors.
    pub fn min(&self) -> Scalar {
        unsafe { ffi::cpSlideJointGetMin(self.ptr()) }
    }
    /// Set the distance the joint will maintain between the two anchors.
    pub fn set_min(&mut self, min: Scalar){
        unsafe { ffi::cpSlideJointSetMin(self.mut_ptr(), min); }
    }

    /// Get the maximum distance the joint will maintain between the two anchors.
    pub fn max(&self) -> Scalar {
        unsafe { ffi::cpSlideJointGetMax(self.ptr()) }
    }
    /// Set the maximum distance the joint will maintain between the two anchors.
    pub fn set_max(&mut self, max: Scalar){
        unsafe { ffi::cpSlideJointSetMax(self.mut_ptr(), max); }
    }
}

pub struct PivotJoint(ffi::cpPivotJoint);
impl_constraint!(PivotJoint);

impl PivotJoint {
    pub fn new(anchor_a: Pnt2, anchor_b: Pnt2) -> Self {
        PivotJoint(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpPivotJointInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), anchor_a, anchor_b
            );
            data
        })
    }

    /// Get the location of the first anchor relative to the first body.
    pub fn anchor_a(&self) -> Pnt2 {
        unsafe { ffi::cpPivotJointGetAnchorA(self.ptr()) }
    }
    /// Set the location of the first anchor relative to the first body.
    pub fn set_anchor_a(&mut self, a: Pnt2){
        unsafe { ffi::cpPivotJointSetAnchorA(self.mut_ptr(), a); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpPivotJointGetAnchorB(self.ptr()) }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpPivotJointSetAnchorB(self.mut_ptr(), b); }
    }
}

pub struct GrooveJoint(ffi::cpGrooveJoint);
impl_constraint!(GrooveJoint);

impl GrooveJoint {
    pub fn new(groove_a: Pnt2, groove_b: Pnt2, anchor_b: Pnt2) -> Self {
        GrooveJoint(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpGrooveJointInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), groove_a, groove_b, anchor_b
            );
            data
        })
    }

    /// Get the first endpoint of the groove relative to the first body.
    pub fn groove_a(&self) -> Pnt2 {
        unsafe { ffi::cpGrooveJointGetGrooveA(self.ptr()) }
    }
    /// Set the first endpoint of the groove relative to the first body.
    pub fn set_groove_a(&mut self, a: Pnt2){
        unsafe { ffi::cpGrooveJointSetGrooveA(self.mut_ptr(), a); }
    }

    /// Get the second endpoint of the groove relative to the first body.
    pub fn groove_b(&self) -> Pnt2 {
        unsafe { ffi::cpGrooveJointGetGrooveB(self.ptr()) }
    }
    /// Set the second endpoint of the groove relative to the first body.
    pub fn set_groove_b(&mut self, b: Pnt2){
        unsafe { ffi::cpGrooveJointSetGrooveB(self.mut_ptr(), b); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpGrooveJointGetAnchorB(self.ptr()) }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpGrooveJointSetAnchorB(self.mut_ptr(), b); }
    }
}

pub struct DampedSpring(ffi::cpDampedSpring);
impl_constraint!(DampedSpring);

pub type DampedSpringForceFunc = extern "C" fn(spring: &DampedSpring, dist: Scalar) -> Scalar;

impl DampedSpring {
    pub fn new(anchor_a: Pnt2, 
               anchor_b: Pnt2, 
               rest_length: Scalar, 
               stiffness: Scalar, 
               damping: Scalar) 
               -> Self 
    {
        DampedSpring(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpDampedSpringInit(
                &mut data, ptr::null_mut(), ptr::null_mut(),
                anchor_a, anchor_b,
                rest_length, stiffness, damping,
            );
            data
        })
    }

    /// Get the location of the first anchor relative to the first body.
    pub fn anchor_a(&self) -> Pnt2 {
        unsafe { ffi::cpDampedSpringGetAnchorA(self.ptr()) }
    }
    /// Set the location of the first anchor relative to the first body.
    pub fn set_anchor_a(&mut self, a: Pnt2){
        unsafe { ffi::cpDampedSpringSetAnchorA(self.mut_ptr(), a); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpDampedSpringGetAnchorB(self.ptr()) }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpDampedSpringSetAnchorB(self.mut_ptr(), b); }
    }

    /// Get the rest length of the spring.
    pub fn rest_length(&self) -> Scalar {
        unsafe { ffi::cpDampedSpringGetRestLength(self.ptr()) }
    }
    /// Set the rest length of the spring.
    pub fn set_rest_length(&mut self, rest_length: Scalar){
        unsafe { ffi::cpDampedSpringSetRestLength(self.mut_ptr(), rest_length); }
    }

    /// Get the stiffness of the spring in force/distance.
    pub fn stiffness(&self) -> Scalar {
        unsafe { ffi::cpDampedSpringGetStiffness(self.ptr()) }
    }
    /// Set the stiffness of the spring in force/distance.
    pub fn set_stiffness(&mut self, stiffness: Scalar){
        unsafe { ffi::cpDampedSpringSetStiffness(self.mut_ptr(), stiffness); }
    }

    /// Get the damping of the spring.
    pub fn damping(&self) -> Scalar {
        unsafe { ffi::cpDampedSpringGetStiffness(self.ptr()) }
    }
    /// Set the damping of the spring.
    pub fn set_damping(&mut self, damping: Scalar){
        unsafe { ffi::cpDampedSpringSetStiffness(self.mut_ptr(), damping); }
    }

    pub fn set_spring_force_func(&mut self, func: DampedSpringForceFunc){
        unsafe {
            ffi::cpDampedSpringSetSpringForceFunc(self.mut_ptr(), mem::transmute(func))
        }
    }
}

pub struct DampedRotarySpring(ffi::cpDampedRotarySpring);
impl_constraint!(DampedRotarySpring);

pub type DampedRotarySpringTorqueFunc = 
    extern "C" fn(spring: &DampedRotarySpring, relative_angle: Scalar) -> Scalar;

impl DampedRotarySpring {
    pub fn new(rest_angle: Scalar, stiffness: Scalar,  damping: Scalar) -> Self {
        DampedRotarySpring(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpDampedRotarySpringInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), rest_angle, stiffness, damping,
            );
            data
        })
    }

    pub fn rest_angle(&self) -> Scalar {
        unsafe { ffi::cpDampedRotarySpringGetRestAngle(self.ptr()) }
    }
    pub fn set_rest_angle(&mut self, rest_angle: Scalar){
        unsafe { ffi::cpDampedRotarySpringSetRestAngle(self.mut_ptr(), rest_angle); }
    }

    /// Get the stiffness of the spring in force/distance.
    pub fn stiffness(&self) -> Scalar {
        unsafe { ffi::cpDampedRotarySpringGetStiffness(self.ptr()) }
    }
    /// Set the stiffness of the spring in force/distance.
    pub fn set_stiffness(&mut self, stiffness: Scalar){
        unsafe { ffi::cpDampedRotarySpringSetStiffness(self.mut_ptr(), stiffness); }
    }

    /// Get the damping of the spring.
    pub fn damping(&self) -> Scalar {
        unsafe { ffi::cpDampedRotarySpringGetStiffness(self.ptr()) }
    }
    /// Set the damping of the spring.
    pub fn set_damping(&mut self, damping: Scalar){
        unsafe { ffi::cpDampedRotarySpringSetStiffness(self.mut_ptr(), damping); }
    }

    pub fn set_spring_force_func(&mut self, func: DampedRotarySpringTorqueFunc){
        unsafe {
            ffi::cpDampedRotarySpringSetSpringTorqueFunc(self.mut_ptr(), mem::transmute(func))
        }
    }
}

pub struct RotaryLimitJoint(ffi::cpRotaryLimitJoint);
impl_constraint!(RotaryLimitJoint);

impl RotaryLimitJoint {
    pub fn new(min: Scalar, max: Scalar) -> Self {
        RotaryLimitJoint(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpRotaryLimitJointInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), min, max
            );
            data
        })
    }
    
    /// Get the distance the joint will maintain between the two anchors.
    pub fn min(&self) -> Scalar {
        unsafe { ffi::cpRotaryLimitJointGetMin(self.ptr()) }
    }
    /// Set the distance the joint will maintain between the two anchors.
    pub fn set_min(&mut self, min: Scalar){
        unsafe { ffi::cpRotaryLimitJointSetMin(self.mut_ptr(), min); }
    }

    /// Get the maximum distance the joint will maintain between the two anchors.
    pub fn max(&self) -> Scalar {
        unsafe { ffi::cpRotaryLimitJointGetMax(self.ptr()) }
    }
    /// Set the maximum distance the joint will maintain between the two anchors.
    pub fn set_max(&mut self, max: Scalar){
        unsafe { ffi::cpRotaryLimitJointSetMax(self.mut_ptr(), max); }
    }
}

pub struct RatchetJoint(ffi::cpRatchetJoint);
impl_constraint!(RatchetJoint);

impl RatchetJoint {
    pub fn new(phase: Scalar, ratchet: Scalar) -> Self {
        RatchetJoint(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpRatchetJointInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), phase, ratchet
            );
            data
        })
    }
    
    /// Get the angle of the current ratchet tooth.
    pub fn angle(&self) -> Scalar {
        unsafe { ffi::cpRatchetJointGetAngle(self.ptr()) }
    }
    /// Set the angle of the current ratchet tooth.
    pub fn set_angle(&mut self, angle: Scalar){
        unsafe { ffi::cpRatchetJointSetAngle(self.mut_ptr(), angle); }
    }

    /// Get the phase offset of the ratchet.
    pub fn phase(&self) -> Scalar {
        unsafe { ffi::cpRatchetJointGetPhase(self.ptr()) }
    }
    /// Get the phase offset of the ratchet.
    pub fn set_phase(&mut self, phase: Scalar){
        unsafe { ffi::cpRatchetJointSetPhase(self.mut_ptr(), phase); }
    }

    /// Get the angular distance of each ratchet.
    pub fn ratchet(&self) -> Scalar {
        unsafe { ffi::cpRatchetJointGetRatchet(self.ptr()) }
    }
    /// Set the angular distance of each ratchet.
    pub fn set_ratchet(&mut self, ratchet: Scalar){
        unsafe { ffi::cpRatchetJointSetRatchet(self.mut_ptr(), ratchet); }
    }
}

pub struct GearJoint(ffi::cpGearJoint);
impl_constraint!(GearJoint);

impl GearJoint {
    pub fn new(phase: Scalar, ratio: Scalar) -> Self {
        GearJoint(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpGearJointInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), phase, ratio
            );
            data
        })
    }

    /// Get the phase offset of the gears.
    pub fn phase(&self) -> Scalar {
        unsafe { ffi::cpGearJointGetPhase(self.ptr()) }
    }
    /// Set the phase offset of the gears.
    pub fn set_phase(&mut self, phase: Scalar){
        unsafe { ffi::cpGearJointSetPhase(self.mut_ptr(), phase); }
    }

    /// Get the angular distance of each ratchet.
    pub fn ratio(&self) -> Scalar {
        unsafe { ffi::cpGearJointGetRatio(self.ptr()) }
    }
    /// Set the ratio of a gear joint.
    pub fn set_ratio(&mut self, ratio: Scalar){
        unsafe { ffi::cpGearJointSetRatio(self.mut_ptr(), ratio); }
    }
}

pub struct SimpleMotor(ffi::cpSimpleMotor);
impl_constraint!(SimpleMotor);

impl SimpleMotor {
    pub fn new(rate: Scalar) -> Self {
        SimpleMotor(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpSimpleMotorInit(
                &mut data, ptr::null_mut(), ptr::null_mut(), rate
            );
            data
        })
    }

    /// Get the rate of the motor.
    pub fn rate(&self) -> Scalar {
        unsafe { ffi::cpSimpleMotorGetRate(self.ptr()) }
    }
    /// Set the rate of the motor.
    pub fn set_rate(&mut self, rate: Scalar){
        unsafe { ffi::cpSimpleMotorSetRate(self.mut_ptr(), rate); }
    }
}
