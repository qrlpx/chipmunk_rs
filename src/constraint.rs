use ::{Scalar, Pnt2};
use body::BodyHandle;
use ffi;

use std::{mem, ptr};

cp_object!{ (Constraint, ConstraintHandle, ConstraintDowncast, ConstraintDowncastRef, ConstraintDowncastMut): ffi::cpConstraint {
    drop: |self_: &mut Constraint| {
        unsafe { ffi::cpConstraintDestroy(self_.as_mut_ptr()); }
    },
    downcast: |self_: Box<Constraint>| { 
        unsafe {
            let ptr = self_.as_ptr();
            if ffi::cpConstraintIsPinJoint(ptr) == 1 {
                ConstraintDowncast::Pin(mem::transmute(self_))
            } else if ffi::cpConstraintIsSlideJoint(ptr) == 1{
                ConstraintDowncast::Slide(mem::transmute(self_))
            } else if ffi::cpConstraintIsPivotJoint(ptr) == 1{
                ConstraintDowncast::Pivot(mem::transmute(self_))
            } else if ffi::cpConstraintIsGrooveJoint(ptr) == 1 {
                ConstraintDowncast::Groove(mem::transmute(self_))
            } else if ffi::cpConstraintIsDampedSpring(ptr) == 1 {
                ConstraintDowncast::DampedSpring(mem::transmute(self_))
            } else if ffi::cpConstraintIsDampedRotarySpring(ptr) == 1 {
                ConstraintDowncast::DampedRotarySpring(mem::transmute(self_))
            } else if ffi::cpConstraintIsRatchetJoint(ptr) == 1 {
                ConstraintDowncast::Ratchet(mem::transmute(self_))
            } else if ffi::cpConstraintIsGearJoint(ptr) == 1 {
                ConstraintDowncast::Gear(mem::transmute(self_))
            } else if ffi::cpConstraintIsSimpleMotor(ptr) == 1 {
                ConstraintDowncast::SimpleMotor(mem::transmute(self_))
            } else {
                unreachable!()
            }
        }
    },
    variants: {
        (PinJoint, Pin): ffi::cpPinJoint,
        (SlideJoint, Slide): ffi::cpSlideJoint,
        (PivotJoint, Pivot): ffi::cpPivotJoint,
        (GrooveJoint, Groove): ffi::cpGrooveJoint,
        (DampedSpring, DampedSpring): ffi::cpDampedSpring,
        (DampedRotarySpring, DampedRotarySpring): ffi::cpDampedRotarySpring,
        (RotaryLimitJoint, RotaryLimit): ffi::cpRotaryLimitJoint,
        (RatchetJoint, Ratchet): ffi::cpRatchetJoint,
        (GearJoint, Gear): ffi::cpGearJoint,
        (SimpleMotor, SimpleMotor): ffi::cpSimpleMotor,
    }
} }

impl Constraint {
    /// TODO explore chipmunk code some more... what happens if we call this method
    /// after removing a Constraint from Space?
    pub unsafe fn set_bodies(&mut self, bodies: Option<(BodyHandle, BodyHandle)>){
        match bodies {
            Some((a, b)) => {
                (*self.as_mut_ptr()).a = a.as_mut_ptr();
                (*self.as_mut_ptr()).b = b.as_mut_ptr();
            }
            None => {
                (*self.as_mut_ptr()).a = ptr::null_mut();
                (*self.as_mut_ptr()).b = ptr::null_mut();
            }
        }
    }

    pub fn is_attached(&self) -> bool {
        unsafe { !(*self.as_ptr()).a.is_null() && !(*self.as_ptr()).b.is_null() }
    }

    pub fn bodies(&self) -> Option<(BodyHandle, BodyHandle)> {
        if self.is_attached() {
            let a = BodyHandle::new(unsafe { (*self.as_ptr()).a });
            let b = BodyHandle::new(unsafe { (*self.as_ptr()).b });
            Some((a, b))
        } else {
            None
        }
    }

    /// Get the maximum force that this constraint is allowed to use.
    pub fn max_force(&self) -> Scalar {
        unsafe { ffi::cpConstraintGetMaxForce(self.as_ptr()) }
    }
    /// Set the maximum force that this constraint is allowed to use. (defaults to INFINITY)
    pub fn set_max_force(&mut self, max_force: Scalar){
        unsafe { ffi::cpConstraintSetMaxForce(self.as_mut_ptr(), max_force); }
    }

    /// Get rate at which joint error is corrected.
    pub fn error_bias(&self) -> Scalar {
        unsafe { ffi::cpConstraintGetErrorBias(self.as_ptr()) }
    }
    /// Set rate at which joint error is corrected.
    /// Defaults to pow(1.0 - 0.1, 60.0) meaning that it will
    /// correct 10% of the error every 1/60th of a second.
    pub fn set_error_bias(&mut self, error_bias: Scalar){
        unsafe { ffi::cpConstraintSetErrorBias(self.as_mut_ptr(), error_bias); }
    }

    /// Get the maximum rate at which joint error is corrected.
    pub fn max_bias(&self) -> Scalar {
        unsafe { ffi::cpConstraintGetMaxBias(self.as_ptr()) }
    }
    /// Set the maximum rate at which joint error is corrected. (defaults to INFINITY)
    pub fn set_max_bias(&mut self, max_bias: Scalar){
        unsafe { ffi::cpConstraintSetMaxBias(self.as_mut_ptr(), max_bias); }
    }

    /// Get if the two bodies connected by the constraint are allowed to collide or not.
    pub fn collide_bodies(&self) -> bool {
        unsafe { ffi::cpConstraintGetCollideBodies(self.as_ptr()) == 1 }
    }
    /// Set if the two bodies connected by the constraint are allowed to collide or not. 
    /// (defaults to cpFalse)
    pub fn set_collide_bodies(&mut self, collide_bodies: bool) {
        let c = if collide_bodies { 1 } else { 0 };
        unsafe { ffi::cpConstraintSetCollideBodies(self.as_mut_ptr(), c); }
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
        unsafe { ffi::cpConstraintGetImpulse(self.as_ptr() as *mut _) } // TODO: fix const
    }

}

impl PinJoint {
    pub fn new(anchor_a: Pnt2, anchor_b: Pnt2) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpPinJointInit(&mut data, ptr::null_mut(), ptr::null_mut(), anchor_a.to_vec(), anchor_b.to_vec());
            PinJoint{ __data: data }
        }
    }

    /// Get the location of the first anchor relative to the first body.
    pub fn anchor_a(&self) -> Pnt2 {
        unsafe { ffi::cpPinJointGetAnchorA(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the first anchor relative to the first body.
    pub fn set_anchor_a(&mut self, a: Pnt2){
        unsafe { ffi::cpPinJointSetAnchorA(self.as_mut_ptr(), a.to_vec()); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpPinJointGetAnchorB(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpPinJointSetAnchorB(self.as_mut_ptr(), b.to_vec()); }
    }
    
    /// Get the distance the joint will maintain between the two anchors.
    pub fn dist(&self) -> Scalar {
        unsafe { ffi::cpPinJointGetDist(self.as_ptr()) }
    }
    /// Set the distance the joint will maintain between the two anchors.
    pub fn set_dist(&mut self, dist: Scalar){
        unsafe { ffi::cpPinJointSetDist(self.as_mut_ptr(), dist); }
    }
}

impl SlideJoint {
    pub fn new(anchor_a: Pnt2, anchor_b: Pnt2, min: Scalar, max: Scalar) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpSlideJointInit(&mut data, ptr::null_mut(), ptr::null_mut(), anchor_a.to_vec(), anchor_b.to_vec(), min, max);
            SlideJoint{ __data: data }
        }
    }

    /// Get the location of the first anchor relative to the first body.
    pub fn anchor_a(&self) -> Pnt2 {
        unsafe { ffi::cpSlideJointGetAnchorA(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the first anchor relative to the first body.
    pub fn set_anchor_a(&mut self, a: Pnt2){
        unsafe { ffi::cpSlideJointSetAnchorA(self.as_mut_ptr(), a.to_vec()); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpSlideJointGetAnchorB(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpSlideJointSetAnchorB(self.as_mut_ptr(), b.to_vec()); }
    }
    
    /// Get the distance the joint will maintain between the two anchors.
    pub fn min(&self) -> Scalar {
        unsafe { ffi::cpSlideJointGetMin(self.as_ptr()) }
    }
    /// Set the distance the joint will maintain between the two anchors.
    pub fn set_min(&mut self, min: Scalar){
        unsafe { ffi::cpSlideJointSetMin(self.as_mut_ptr(), min); }
    }

    /// Get the maximum distance the joint will maintain between the two anchors.
    pub fn max(&self) -> Scalar {
        unsafe { ffi::cpSlideJointGetMax(self.as_ptr()) }
    }
    /// Set the maximum distance the joint will maintain between the two anchors.
    pub fn set_max(&mut self, max: Scalar){
        unsafe { ffi::cpSlideJointSetMax(self.as_mut_ptr(), max); }
    }
}

impl PivotJoint {
    pub fn new(anchor_a: Pnt2, anchor_b: Pnt2) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpPivotJointInit(&mut data, ptr::null_mut(), ptr::null_mut(), anchor_a.to_vec(), anchor_b.to_vec());
            PivotJoint{ __data: data }
        }
    }

    /// Get the location of the first anchor relative to the first body.
    pub fn anchor_a(&self) -> Pnt2 {
        unsafe { ffi::cpPivotJointGetAnchorA(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the first anchor relative to the first body.
    pub fn set_anchor_a(&mut self, a: Pnt2){
        unsafe { ffi::cpPivotJointSetAnchorA(self.as_mut_ptr(), a.to_vec()); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpPivotJointGetAnchorB(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpPivotJointSetAnchorB(self.as_mut_ptr(), b.to_vec()); }
    }
}

impl GrooveJoint {
    pub fn new(groove_a: Pnt2, groove_b: Pnt2, anchor_b: Pnt2) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpGrooveJointInit(&mut data, ptr::null_mut(), ptr::null_mut(), groove_a.to_vec(), groove_b.to_vec(), anchor_b.to_vec());
            GrooveJoint{ __data: data }
        }
    }

    /// Get the first endpoint of the groove relative to the first body.
    pub fn groove_a(&self) -> Pnt2 {
        unsafe { ffi::cpGrooveJointGetGrooveA(self.as_ptr()).to_pnt() }
    }
    /// Set the first endpoint of the groove relative to the first body.
    pub fn set_groove_a(&mut self, a: Pnt2){
        unsafe { ffi::cpGrooveJointSetGrooveA(self.as_mut_ptr(), a.to_vec()); }
    }

    /// Get the second endpoint of the groove relative to the first body.
    pub fn groove_b(&self) -> Pnt2 {
        unsafe { ffi::cpGrooveJointGetGrooveB(self.as_ptr()).to_pnt() }
    }
    /// Set the second endpoint of the groove relative to the first body.
    pub fn set_groove_b(&mut self, b: Pnt2){
        unsafe { ffi::cpGrooveJointSetGrooveB(self.as_mut_ptr(), b.to_vec()); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpGrooveJointGetAnchorB(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpGrooveJointSetAnchorB(self.as_mut_ptr(), b.to_vec()); }
    }
}

pub type DampedSpringForceFunc = extern "C" fn(spring: &DampedSpring, dist: Scalar) -> Scalar;

impl DampedSpring {
    pub fn new(anchor_a: Pnt2, anchor_b: Pnt2, rest_length: Scalar, stiffness: Scalar, damping: Scalar)  -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpDampedSpringInit(&mut data, ptr::null_mut(), ptr::null_mut(), anchor_a.to_vec(), anchor_b.to_vec(), rest_length, stiffness, damping);
            DampedSpring{ __data: data }
        }
    }

    /// Get the location of the first anchor relative to the first body.
    pub fn anchor_a(&self) -> Pnt2 {
        unsafe { ffi::cpDampedSpringGetAnchorA(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the first anchor relative to the first body.
    pub fn set_anchor_a(&mut self, a: Pnt2){
        unsafe { ffi::cpDampedSpringSetAnchorA(self.as_mut_ptr(), a.to_vec()); }
    }
    
    /// Get the location of the second anchor relative to the second body.
    pub fn anchor_b(&self) -> Pnt2 {
        unsafe { ffi::cpDampedSpringGetAnchorB(self.as_ptr()).to_pnt() }
    }
    /// Set the location of the second anchor relative to the second body.
    pub fn set_anchor_b(&mut self, b: Pnt2){
        unsafe { ffi::cpDampedSpringSetAnchorB(self.as_mut_ptr(), b.to_vec()); }
    }

    /// Get the rest length of the spring.
    pub fn rest_length(&self) -> Scalar {
        unsafe { ffi::cpDampedSpringGetRestLength(self.as_ptr()) }
    }
    /// Set the rest length of the spring.
    pub fn set_rest_length(&mut self, rest_length: Scalar){
        unsafe { ffi::cpDampedSpringSetRestLength(self.as_mut_ptr(), rest_length); }
    }

    /// Get the stiffness of the spring in force/distance.
    pub fn stiffness(&self) -> Scalar {
        unsafe { ffi::cpDampedSpringGetStiffness(self.as_ptr()) }
    }
    /// Set the stiffness of the spring in force/distance.
    pub fn set_stiffness(&mut self, stiffness: Scalar){
        unsafe { ffi::cpDampedSpringSetStiffness(self.as_mut_ptr(), stiffness); }
    }

    /// Get the damping of the spring.
    pub fn damping(&self) -> Scalar {
        unsafe { ffi::cpDampedSpringGetStiffness(self.as_ptr()) }
    }
    /// Set the damping of the spring.
    pub fn set_damping(&mut self, damping: Scalar){
        unsafe { ffi::cpDampedSpringSetStiffness(self.as_mut_ptr(), damping); }
    }

    pub fn set_spring_force_func(&mut self, func: DampedSpringForceFunc){
        unsafe { ffi::cpDampedSpringSetSpringForceFunc(self.as_mut_ptr(), mem::transmute(func)) }
    }
}


pub type DampedRotarySpringTorqueFunc = extern "C" fn(spring: &DampedRotarySpring, relative_angle: Scalar) -> Scalar;

impl DampedRotarySpring {
    pub fn new(rest_angle: Scalar, stiffness: Scalar,  damping: Scalar) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpDampedRotarySpringInit(&mut data, ptr::null_mut(), ptr::null_mut(), rest_angle, stiffness, damping);
            DampedRotarySpring{ __data: data }
        }
    }

    pub fn rest_angle(&self) -> Scalar {
        unsafe { ffi::cpDampedRotarySpringGetRestAngle(self.as_ptr()) }
    }
    pub fn set_rest_angle(&mut self, rest_angle: Scalar){
        unsafe { ffi::cpDampedRotarySpringSetRestAngle(self.as_mut_ptr(), rest_angle); }
    }

    /// Get the stiffness of the spring in force/distance.
    pub fn stiffness(&self) -> Scalar {
        unsafe { ffi::cpDampedRotarySpringGetStiffness(self.as_ptr()) }
    }
    /// Set the stiffness of the spring in force/distance.
    pub fn set_stiffness(&mut self, stiffness: Scalar){
        unsafe { ffi::cpDampedRotarySpringSetStiffness(self.as_mut_ptr(), stiffness); }
    }

    /// Get the damping of the spring.
    pub fn damping(&self) -> Scalar {
        unsafe { ffi::cpDampedRotarySpringGetStiffness(self.as_ptr()) }
    }
    /// Set the damping of the spring.
    pub fn set_damping(&mut self, damping: Scalar){
        unsafe { ffi::cpDampedRotarySpringSetStiffness(self.as_mut_ptr(), damping); }
    }

    pub fn set_spring_force_func(&mut self, func: DampedRotarySpringTorqueFunc){
        unsafe { ffi::cpDampedRotarySpringSetSpringTorqueFunc(self.as_mut_ptr(), mem::transmute(func)) }
    }
}

impl RotaryLimitJoint {
    pub fn new(min: Scalar, max: Scalar) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpRotaryLimitJointInit(&mut data, ptr::null_mut(), ptr::null_mut(), min, max);
            RotaryLimitJoint{ __data: data }
        }
    }
    
    /// Get the distance the joint will maintain between the two anchors.
    pub fn min(&self) -> Scalar {
        unsafe { ffi::cpRotaryLimitJointGetMin(self.as_ptr()) }
    }
    /// Set the distance the joint will maintain between the two anchors.
    pub fn set_min(&mut self, min: Scalar){
        unsafe { ffi::cpRotaryLimitJointSetMin(self.as_mut_ptr(), min); }
    }

    /// Get the maximum distance the joint will maintain between the two anchors.
    pub fn max(&self) -> Scalar {
        unsafe { ffi::cpRotaryLimitJointGetMax(self.as_ptr()) }
    }
    /// Set the maximum distance the joint will maintain between the two anchors.
    pub fn set_max(&mut self, max: Scalar){
        unsafe { ffi::cpRotaryLimitJointSetMax(self.as_mut_ptr(), max); }
    }
}

impl RatchetJoint {
    pub fn new(phase: Scalar, ratchet: Scalar) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpRatchetJointInit(&mut data, ptr::null_mut(), ptr::null_mut(), phase, ratchet);
            RatchetJoint{ __data: data }
        }
    }
    
    /// Get the angle of the current ratchet tooth.
    pub fn angle(&self) -> Scalar {
        unsafe { ffi::cpRatchetJointGetAngle(self.as_ptr()) }
    }
    /// Set the angle of the current ratchet tooth.
    pub fn set_angle(&mut self, angle: Scalar){
        unsafe { ffi::cpRatchetJointSetAngle(self.as_mut_ptr(), angle); }
    }

    /// Get the phase offset of the ratchet.
    pub fn phase(&self) -> Scalar {
        unsafe { ffi::cpRatchetJointGetPhase(self.as_ptr()) }
    }
    /// Get the phase offset of the ratchet.
    pub fn set_phase(&mut self, phase: Scalar){
        unsafe { ffi::cpRatchetJointSetPhase(self.as_mut_ptr(), phase); }
    }

    /// Get the angular distance of each ratchet.
    pub fn ratchet(&self) -> Scalar {
        unsafe { ffi::cpRatchetJointGetRatchet(self.as_ptr()) }
    }
    /// Set the angular distance of each ratchet.
    pub fn set_ratchet(&mut self, ratchet: Scalar){
        unsafe { ffi::cpRatchetJointSetRatchet(self.as_mut_ptr(), ratchet); }
    }
}

impl GearJoint {
    pub fn new(phase: Scalar, ratio: Scalar) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpGearJointInit(&mut data, ptr::null_mut(), ptr::null_mut(), phase, ratio);
            GearJoint{ __data: data } 
        }
    }

    /// Get the phase offset of the gears.
    pub fn phase(&self) -> Scalar {
        unsafe { ffi::cpGearJointGetPhase(self.as_ptr()) }
    }
    /// Set the phase offset of the gears.
    pub fn set_phase(&mut self, phase: Scalar){
        unsafe { ffi::cpGearJointSetPhase(self.as_mut_ptr(), phase); }
    }

    /// Get the angular distance of each ratchet.
    pub fn ratio(&self) -> Scalar {
        unsafe { ffi::cpGearJointGetRatio(self.as_ptr()) }
    }
    /// Set the ratio of a gear joint.
    pub fn set_ratio(&mut self, ratio: Scalar){
        unsafe { ffi::cpGearJointSetRatio(self.as_mut_ptr(), ratio); }
    }
}

impl SimpleMotor {
    pub fn new(rate: Scalar) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpSimpleMotorInit(&mut data, ptr::null_mut(), ptr::null_mut(), rate);
            SimpleMotor{ __data: data }
        }
    }

    /// Get the rate of the motor.
    pub fn rate(&self) -> Scalar {
        unsafe { ffi::cpSimpleMotorGetRate(self.as_ptr()) }
    }
    /// Set the rate of the motor.
    pub fn set_rate(&mut self, rate: Scalar){
        unsafe { ffi::cpSimpleMotorSetRate(self.as_mut_ptr(), rate); }
    }
}

