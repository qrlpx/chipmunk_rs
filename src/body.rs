use ::{Scalar, Pnt2, Vec2, Rot2};
use arbiter::Arbiter;
use constraint::ConstraintHandle;
use shape::ShapeHandle;
use ffi;

use na;

use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
use std::{mem};

cp_object!{ (Body, BodyHandle, BodyDowncast, BodyDowncastRef, BodyDowncastMut): ffi::cpBody {
    drop: |self_: &mut Body| {
        unsafe { ffi::cpBodyDestroy(self_.as_mut_ptr()); }
    },
    downcast: |self_: Box<Body>| { 
        unsafe {
            match ffi::cpBodyGetType(self_.as_ptr() as *mut _) {
                ffi::CP_BODY_TYPE_STATIC => BodyDowncast::Static(mem::transmute(self_)),
                ffi::CP_BODY_TYPE_KINEMATIC => BodyDowncast::Kinematic(mem::transmute(self_)),
                ffi::CP_BODY_TYPE_DYNAMIC => BodyDowncast::Dynamic(mem::transmute(self_)),
                _ => unreachable!()
            }
        }
    },
    variants: {
        (StaticBody, Static): ffi::cpBody,
        (KinematicBody, Kinematic): ffi::cpBody,
        (DynamicBody, Dynamic): ffi::cpBody,
    }
} }

// Wake up any sleeping or idle bodies touching a static body.
//void cpBodyActivateStatic(cpBody *body, cpShape *filter);

// Force a body to fall asleep immediately along with other bodies in a group.
//void cpBodySleepWithGroup(cpBody *body, cpBody *group);

impl Body {
    pub fn position(&self) -> Pnt2 {
        unsafe { ffi::cpBodyGetPosition(self.as_ptr()).to_pnt() }
    }
    pub fn set_position(&mut self, pos: Pnt2){
        unsafe { ffi::cpBodySetPosition(self.as_mut_ptr(), pos.to_vec()); }
    }

    pub fn angle(&self) -> Scalar {
        unsafe { ffi::cpBodyGetAngle(self.as_ptr()) }
    }
    pub fn set_angle(&mut self, a: Scalar){
        unsafe { ffi::cpBodySetAngle(self.as_mut_ptr(), a); }
    }

    pub fn velocity(&self) -> Vec2 {
        unsafe { ffi::cpBodyGetVelocity(self.as_ptr()) }
    }
    pub fn set_velocity(&mut self, v: Vec2){
        unsafe { ffi::cpBodySetVelocity(self.as_mut_ptr(), v); }
    }

    /// Get the force applied to the body for the next time step.
    pub fn force(&self) -> Vec2 {
        unsafe { ffi::cpBodyGetForce(self.as_ptr()) }
    }
    /// Set the force applied to the body for the next time step.
    pub fn set_force(&mut self, f: Vec2){
        unsafe { ffi::cpBodySetForce(self.as_mut_ptr(), f); }
    }

    pub fn angular_velocity(&self) -> Scalar {
        unsafe {ffi::cpBodyGetAngularVelocity(self.as_ptr()) }
    }
    pub fn set_angular_velocity(&mut self, av: Scalar){
        unsafe { ffi::cpBodySetAngularVelocity(self.as_mut_ptr(), av); }
    }

    /// Get the torque applied to the body for the next time step.
    pub fn torque(&self) -> Scalar {
        unsafe { ffi::cpBodyGetTorque(self.as_ptr()) }
    }
    /// Set the torque applied to the body for the next time step.
    pub fn set_torque(&mut self, t: Scalar){
        unsafe { ffi::cpBodySetTorque(self.as_mut_ptr(), t); }
    }

    // Get the rotation matrix of the body. 
    //pub fn rotation(&self) -> Rot2 {
    //    unsafe { ffi::cpBodyGetRotation(self.as_ptr()) }
    //}

    // Set the callback used to update a body's velocity.
    // TODO ffi::cpBodySetVelocityUpdateFunc(&mut selfbody, cpBodyVelocityFunc velocityFunc);

    // Set the callback used to update a body's position.
    // NOTE: It's not generally recommended to override this.
    // TODO ffi::cpBodySetPositionUpdateFunc(&mut selfbody, cpBodyPositionFunc positionFunc);

    // Default velocity integration function..
    //TODO ffi::cpBodyUpdateVelocity(&mut selfbody, cpPnt2 gravity, cpFloat damping, cpFloat dt);

    // Default position integration function.
    //TODO ffi::cpBodyUpdatePosition(&mut selfbody, cpFloat dt);

    /// Convert body relative/local coordinates to absolute/world coordinates.
    pub fn local_to_world(&self, point: Pnt2) -> Pnt2 {
        unsafe { ffi::cpBodyLocalToWorld(self.as_ptr(), point.to_vec()).to_pnt() }
    }
    /// Convert body absolute/world coordinates to relative/local coordinates.
    pub fn world_to_local(&self, point: Pnt2) -> Pnt2 {
        unsafe { ffi::cpBodyWorldToLocal(self.as_ptr(), point.to_vec()).to_pnt() }
    }

    /* TODO
    pub fn arbiters(&self) -> Arbiters {
        Arbiters{ 
            body: self.as_ptr() as *mut _, 
            node: unsafe { ffi::cpBodyArbiterList(self.as_ptr() as *mut _) },
            _phantom: PhantomData,
        }
    }*/

    pub fn constraints(&self) -> Constraints {
        Constraints{
            body: self.as_ptr() as *mut _,
            node: unsafe { ffi::cpBodyConstraintList(self.as_ptr() as *mut _) },
            _phantom: PhantomData,
        }
    }

    pub fn shapes(&self) -> Shapes {
        Shapes{ 
            node: unsafe { ffi::cpBodyShapeList(self.as_ptr() as *mut _) }, 
            _phantom: PhantomData,
        } 
    }
}

impl StaticBody {
    pub fn new() -> Self {
        unsafe { 
            let mut data = mem::zeroed();
            ffi::cpBodyInit(&mut data, 0.0, 0.0);
            ffi::cpBodySetType(&mut data, ffi::CP_BODY_TYPE_STATIC);
            StaticBody{ __data: data }
        }
    }
}

impl KinematicBody {
    pub fn new() -> Self {
        unsafe { 
            let mut data = mem::zeroed();
            ffi::cpBodyInit(&mut data, 0.0, 0.0);
            ffi::cpBodySetType(&mut data, ffi::CP_BODY_TYPE_KINEMATIC);
            KinematicBody{ __data: data }
        }
    }

    /// Apply a force to a body. Both the force and point are expressed in world coordinates.
    pub fn apply_force_at_world_point(&mut self, force: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyForceAtWorldPoint(self.as_mut_ptr(), force, point.to_vec()); }
    }
    /// Apply a force to a body. Both the force and point are expressed in body local coordinates.
    pub fn apply_force_at_local_point(&mut self, force: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyForceAtLocalPoint(self.as_mut_ptr(), force, point.to_vec()); }
    }

    /// Apply an impulse to a body. Both the impulse and point are expressed in world coordinates.
    pub fn apply_impulse_at_world_point(&mut self, impulse: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyImpulseAtWorldPoint(self.as_mut_ptr(), impulse, point.to_vec()); }
    }
    /// Apply an impulse to a body. Both the impulse and point are expressed in body local 
    /// coordinates.
    pub fn apply_impulse_at_local_point(&mut self, impulse: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyImpulseAtLocalPoint(self.as_mut_ptr(), impulse, point.to_vec()); }
    }

    /// Get the velocity on a body (in world units) at a point on the body in world coordinates.
    pub fn velocity_at_world_point(&self, point: Pnt2) -> Vec2 {
        unsafe { ffi::cpBodyGetVelocityAtWorldPoint(self.as_ptr(), point.to_vec()) }
    }
    /// Get the velocity on a body (in world units) at a point on the body in local coordinates.
    pub fn velocity_at_local_point(&self, point: Pnt2) -> Vec2 {
        unsafe { ffi::cpBodyGetVelocityAtLocalPoint(self.as_ptr(), point.to_vec()) }
    }

    /// Get the amount of kinetic energy contained by the body.
    pub fn kinetic_energy(&self) -> Scalar {
        unsafe { ffi::cpBodyKineticEnergy(self.as_ptr()) }
    }
}

impl DynamicBody {
    pub fn new() -> Self {
        unsafe { 
            let mut data = mem::zeroed();
            ffi::cpBodyInit(&mut data, 0.0, 0.0);
            ffi::cpBodySetType(&mut data, ffi::CP_BODY_TYPE_DYNAMIC);
            DynamicBody{ __data: data }
        }
    }

    pub fn wake_up(&mut self){
        unsafe { ffi::cpBodyActivate(self.as_mut_ptr()); }
    }
    pub fn sleep(&mut self){
        unsafe { ffi::cpBodySleep(self.as_mut_ptr()); }
    }
    pub fn is_sleeping(&self) -> bool {
        unsafe { ffi::cpBodyIsSleeping(self.as_ptr()) == 1 }
    }

    pub fn mass(&self) -> Scalar {
        unsafe { ffi::cpBodyGetMass(self.as_ptr()) }
    }
    pub fn set_mass(&mut self, m: Scalar){
        unsafe { ffi::cpBodySetMass(self.as_mut_ptr(), m) }
    }

    /// Get the moment of inertia of the body.
    pub fn moment(&self) -> Scalar {
        unsafe { ffi::cpBodyGetMoment(self.as_ptr()) }
    }
    /// Set the moment of inertia of the body.
    pub fn set_moment(&mut self, i: Scalar){
        unsafe { ffi::cpBodySetMoment(self.as_mut_ptr(), i) }
    }

    /// Get the offset of the center of gravity in body local coordinates.
    pub fn center_of_gravity(&self) -> Pnt2 {
        unsafe { ffi::cpBodyGetCenterOfGravity(self.as_ptr()).to_pnt() }
    }
    /// Set the offset of the center of gravity in body local coordinates.
    pub fn set_center_of_gravity(&mut self, cog: Pnt2){
        unsafe { ffi::cpBodySetCenterOfGravity(self.as_mut_ptr(), cog.to_vec()); }
    }

    // ++++++++++++++++++++
    // duplicated code below, any point in moving it into a trait?
    // ++++++++++++++++++++

    /// Apply a force to a body. Both the force and point are expressed in world coordinates.
    pub fn apply_force_at_world_point(&mut self, force: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyForceAtWorldPoint(self.as_mut_ptr(), force, point.to_vec()); }
    }
    /// Apply a force to a body. Both the force and point are expressed in body local coordinates.
    pub fn apply_force_at_local_point(&mut self, force: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyForceAtLocalPoint(self.as_mut_ptr(), force, point.to_vec()); }
    }

    /// Apply an impulse to a body. Both the impulse and point are expressed in world coordinates.
    pub fn apply_impulse_at_world_point(&mut self, impulse: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyImpulseAtWorldPoint(self.as_mut_ptr(), impulse, point.to_vec()); }
    }
    /// Apply an impulse to a body. Both the impulse and point are expressed in body local 
    /// coordinates.
    pub fn apply_impulse_at_local_point(&mut self, impulse: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyImpulseAtLocalPoint(self.as_mut_ptr(), impulse, point.to_vec()); }
    }

    /// Get the velocity on a body (in world units) at a point on the body in world coordinates.
    pub fn velocity_at_world_point(&self, point: Pnt2) -> Vec2 {
        unsafe { ffi::cpBodyGetVelocityAtWorldPoint(self.as_ptr(), point.to_vec()) }
    }
    /// Get the velocity on a body (in world units) at a point on the body in local coordinates.
    pub fn velocity_at_local_point(&self, point: Pnt2) -> Vec2 {
        unsafe { ffi::cpBodyGetVelocityAtLocalPoint(self.as_ptr(), point.to_vec()) }
    }

    /// Get the amount of kinetic energy contained by the body.
    pub fn kinetic_energy(&self) -> Scalar {
        unsafe { ffi::cpBodyKineticEnergy(self.as_ptr()) }
    }
}

/*
pub struct Arbiters<'a> {
    body: *const ffi::cpBody,
    node: *const ffi::cpArbiter,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for Arbiters<'a> {
    type Item = ArbiterHandle;
    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            if self.node.is_null() { return None; }
            let (mut body_a, mut body_b) = mem::uninitialized();
            ffi::cpArbiterGetBodies_NOSWAP(self.node, &mut body_a, &mut body_b, 0);
            let ret = ArbiterHandle::new(self.node);
            self.node = ffi::cpBodyNextArbiter(self.body as *mut _, self.node as *mut _);
            Some(ret)
        }
    }
}
*/

pub struct Constraints<'a> {
    body: *const ffi::cpBody,
    node: *const ffi::cpConstraint,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for Constraints<'a> {
    type Item = ConstraintHandle;
    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            if self.node.is_null() { return None; }
            let ret = ConstraintHandle::new(self.node);
            self.node = ffi::cpBodyNextConstraint(self.node as *mut _, self.body as *mut _);
            Some(ret)
        }
    }
}

pub struct Shapes<'a> {
    node: *const ffi::cpShape,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for Shapes<'a> {
    type Item = ShapeHandle;
    fn next(&mut self) -> Option<Self::Item> {
        unsafe {
            if self.node.is_null() { return None; }
            let ret = ShapeHandle::new(self.node);
            self.node = ffi::cpBodyNextShape(self.node as *mut _);
            Some(ret)
        }
    }
}
