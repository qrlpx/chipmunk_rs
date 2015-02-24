use ::{Scalar, Pnt2, Vec2, ObjectHandle};
use arbiter::{Arbiter, MutArbiter};
use constraint::ConstraintBase;
use shape::ShapeBase;
use ffi;

use na;

use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};
use std::{mem};

use self::BodyUpcast::*;
pub enum BodyUpcast {
    ToStaticBody(Box<StaticBody>),
    ToKinematicBody(Box<KinematicBody>),
    ToDynamicBody(Box<DynamicBody>),
}

use self::BodyUpcastRef::*;
pub enum BodyUpcastRef<'a> {
    ToStaticBodyRef(&'a StaticBody),
    ToKinematicBodyRef(&'a KinematicBody),
    ToDynamicBodyRef(&'a DynamicBody),
}

use self::BodyUpcastMut::*;
pub enum BodyUpcastMut<'a> {
    ToStaticBodyMut(&'a mut StaticBody),
    ToKinematicBodyMut(&'a mut KinematicBody),
    ToDynamicBodyMut(&'a mut DynamicBody),
}

pub type BodyHandle = ObjectHandle<ffi::cpBody>;

pub struct BodyBase(ffi::cpBody);

unsafe impl Sync for BodyBase {}

impl BodyBase {
    pub unsafe fn from_ptr(ptr: *const ffi::cpBody) -> &'static BodyBase {
        mem::transmute(ptr)
    }
    pub unsafe fn from_mut_ptr(ptr: *mut ffi::cpBody) -> &'static mut BodyBase {
        mem::transmute(ptr)
    }
    pub unsafe fn box_from_ptr(ptr: *const ffi::cpBody) -> Box<BodyBase> {
        mem::transmute(ptr)
    }

    pub fn upcast(_self: Box<Self>) -> BodyUpcast {
        unsafe {
            match ffi::cpBodyGetType(_self.ptr() as *mut _) {
                ffi::CP_BODY_TYPE_STATIC => ToStaticBody(mem::transmute(_self)),
                ffi::CP_BODY_TYPE_KINEMATIC => ToKinematicBody(mem::transmute(_self)),
                ffi::CP_BODY_TYPE_DYNAMIC => ToDynamicBody(mem::transmute(_self)),
                _ => unreachable!()
            }
        }
    }
    pub fn upcast_ref(&self) -> BodyUpcastRef {
        unsafe { mem::transmute(BodyBase::upcast(mem::transmute(self))) }
    }
    pub fn upcast_mut(&mut self) -> BodyUpcastMut {
        unsafe { mem::transmute(BodyBase::upcast(mem::transmute(self))) }
    }

    pub fn ptr(&self) -> *const ffi::cpBody { &self.0 }
    pub fn mut_ptr(&mut self) -> *mut ffi::cpBody { &mut self.0 }
    pub fn handle(&self) -> BodyHandle {
        ObjectHandle::wrap(self.ptr() as *mut _)
    }

// Wake up any sleeping or idle bodies touching a static body.
//void cpBodyActivateStatic(cpBody *body, cpShape *filter);

// Force a body to fall asleep immediately along with other bodies in a group.
//void cpBodySleepWithGroup(cpBody *body, cpBody *group);

    pub fn position(&self) -> Pnt2 {
        unsafe { ffi::cpBodyGetPosition(self.ptr()) }
    }
    pub fn set_position(&mut self, pos: Pnt2){
        unsafe { ffi::cpBodySetPosition(self.mut_ptr(), pos); }
    }

    pub fn velocity(&self) -> Vec2 {
        unsafe { ffi::cpBodyGetVelocity(self.ptr()).to_vec() }
    }
    pub fn set_velocity(&mut self, v: Vec2){
        unsafe { ffi::cpBodySetVelocity(self.mut_ptr(), na::orig::<Pnt2>() + v); }
    }

    /// Get the force applied to the body for the next time step.
    pub fn force(&self) -> Vec2 {
        unsafe { ffi::cpBodyGetForce(self.ptr()).to_vec() }
    }
    /// Set the force applied to the body for the next time step.
    pub fn set_force(&mut self, f: Vec2){
        unsafe { ffi::cpBodySetForce(self.mut_ptr(), na::orig::<Pnt2>() + f); }
    }

    pub fn angle(&self) -> Scalar {
        unsafe { ffi::cpBodyGetAngle(self.ptr()) }
    }
    pub fn set_angle(&mut self, a: Scalar){
        unsafe { ffi::cpBodySetAngle(self.mut_ptr(), a); }
    }

    pub fn angular_velocity(&self) -> Scalar {
        unsafe {ffi::cpBodyGetAngularVelocity(self.ptr()) }
    }
    pub fn set_angular_velocity(&mut self, av: Scalar){
        unsafe { ffi::cpBodySetAngularVelocity(self.mut_ptr(), av); }
    }

    /// Get the torque applied to the body for the next time step.
    pub fn torque(&self) -> Scalar {
        unsafe { ffi::cpBodyGetTorque(self.ptr()) }
    }
    /// Set the torque applied to the body for the next time step.
    pub fn set_torque(&mut self, t: Scalar){
        unsafe { ffi::cpBodySetTorque(self.mut_ptr(), t); }
    }

    // Get the rotation matrix of the body. TODO
    //pub fn rotation(&self) -> Pnt2 {
    //    unsafe { ffi::cpBodyGetRotation(self.ptr()) }
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
        unsafe { ffi::cpBodyLocalToWorld(self.ptr(), point) }
    }
    /// Convert body absolute/world coordinates to relative/local coordinates.
    pub fn world_to_local(&self, point: Pnt2) -> Pnt2 {
        unsafe { ffi::cpBodyWorldToLocal(self.ptr(), point) }
    }

    /// Apply a force to a body. Both the force and point are expressed in world coordinates.
    pub fn apply_force_at_world_point(&mut self, force: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyForceAtWorldPoint(
            self.mut_ptr(), na::orig::<Pnt2>() + force, point
        ); }
    }
    /// Apply a force to a body. Both the force and point are expressed in body local coordinates.
    pub fn apply_force_at_local_point(&mut self, force: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyForceAtLocalPoint(
            self.mut_ptr(), na::orig::<Pnt2>() + force, point
        ); }
    }

    /// Apply an impulse to a body. Both the impulse and point are expressed in world coordinates.
    pub fn apply_impulse_at_world_point(&mut self, impulse: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyImpulseAtWorldPoint(
            self.mut_ptr(), na::orig::<Pnt2>() + impulse, point
        ); }
    }
    /// Apply an impulse to a body. Both the impulse and point are expressed in body local 
    /// coordinates.
    pub fn apply_impulse_at_local_point(&mut self, impulse: Vec2, point: Pnt2){
        unsafe { ffi::cpBodyApplyImpulseAtLocalPoint(
            self.mut_ptr(), na::orig::<Pnt2>() + impulse, point
        ); }
    }

    /// Get the velocity on a body (in world units) at a point on the body in world coordinates.
    pub fn velocity_at_world_point(&self, point: Pnt2) -> Vec2 {
        unsafe { ffi::cpBodyGetVelocityAtWorldPoint(self.ptr(), point).to_vec() }
    }
    /// Get the velocity on a body (in world units) at a point on the body in local coordinates.
    pub fn velocity_at_local_point(&self, point: Pnt2) -> Vec2 {
        unsafe { ffi::cpBodyGetVelocityAtLocalPoint(self.ptr(), point).to_vec() }
    }

    /// Get the amount of kinetic energy contained by the body.
    pub fn kinetic_energy(&self) -> Scalar {
        unsafe { ffi::cpBodyKineticEnergy(self.ptr()) }
    }

    pub fn arbiters(&self) -> Arbiters {
        Arbiters{ 
            body: self.ptr() as *mut _, 
            node: unsafe { ffi::cpBodyArbiterList(self.ptr() as *mut _) },
            _phantom: PhantomData,
        }
    }
    pub fn mut_arbiters(&mut self) -> MutArbiters {
        MutArbiters(self.arbiters())
    }

    pub fn constraints(&self) -> Constraints {
        Constraints{
            body: self.ptr() as *mut _,
            node: unsafe { ffi::cpBodyConstraintList(self.ptr() as *mut _) },
            _phantom: PhantomData,
        }
    }
    pub fn mut_constraints(&mut self) -> MutConstraints {
        MutConstraints(self.constraints())
    }

    pub fn shapes(&self) -> Shapes {
        Shapes{ 
            node: unsafe { ffi::cpBodyShapeList(self.ptr() as *mut _) }, 
            _phantom: PhantomData,
        } 
    }
    pub fn mut_shape_iter(&mut self) -> MutShapes {
        MutShapes(self.shapes())
    }
}

impl Drop for BodyBase {
    fn drop(&mut self){
        unsafe { ffi::cpBodyDestroy(self.mut_ptr()); }
    }
}

macro_rules! impl_body {
    ($ty:ty) => (
        unsafe impl Sync for $ty {}
        impl Deref for $ty {
            type Target = BodyBase;
            fn deref(&self) -> &BodyBase { 
                unsafe { mem::transmute(self) }
            }
        }
        impl DerefMut for $ty {
            fn deref_mut(&mut self) -> &mut BodyBase { 
                unsafe { mem::transmute(self) }
            }
        }
        impl Drop for $ty {
            fn drop(&mut self){
                unsafe { ffi::cpBodyDestroy(self.mut_ptr()); }
            }
        }
    );
}

pub struct StaticBody(ffi::cpBody);
impl_body!(StaticBody);

impl StaticBody {
    pub fn new() -> Self {
        StaticBody(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpBodyInit(&mut data, 0.0, 0.0);
            ffi::cpBodySetType(&mut data, ffi::CP_BODY_TYPE_STATIC);
            data
        })
    }
}

pub struct KinematicBody(ffi::cpBody);
impl_body!(KinematicBody);

impl KinematicBody {
    pub fn new() -> Self {
        KinematicBody(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpBodyInit(&mut data, 0.0, 0.0);
            ffi::cpBodySetType(&mut data, ffi::CP_BODY_TYPE_KINEMATIC);
            data
        })
    }
}

pub struct DynamicBody(ffi::cpBody);
impl_body!(DynamicBody);

impl DynamicBody {
    pub fn new() -> Self {
        DynamicBody(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpBodyInit(&mut data, 0.0, 0.0);
            ffi::cpBodySetType(&mut data, ffi::CP_BODY_TYPE_DYNAMIC);
            data
        })
    }

    pub fn wake_up(&mut self){
        unsafe { ffi::cpBodyActivate(self.mut_ptr()); }
    }
    pub fn sleep(&mut self){
        unsafe { ffi::cpBodySleep(self.mut_ptr()); }
    }
    pub fn is_sleeping(&self) -> bool {
        unsafe { ffi::cpBodyIsSleeping(self.ptr()) == 1 }
    }

    pub fn mass(&self) -> Scalar {
        unsafe { ffi::cpBodyGetMass(self.ptr()) }
    }
    pub fn set_mass(&mut self, m: Scalar){
        unsafe { ffi::cpBodySetMass(self.mut_ptr(), m) }
    }

    /// Get the moment of inertia of the body.
    pub fn moment(&self) -> Scalar {
        unsafe { ffi::cpBodyGetMoment(self.ptr()) }
    }
    /// Set the moment of inertia of the body.
    pub fn set_moment(&mut self, i: Scalar){
        unsafe { ffi::cpBodySetMoment(self.mut_ptr(), i) }
    }

    /// Get the offset of the center of gravity in body local coordinates.
    pub fn center_of_gravity(&self) -> Pnt2 {
        unsafe { ffi::cpBodyGetCenterOfGravity(self.ptr()) }
    }
    /// Set the offset of the center of gravity in body local coordinates.
    pub fn set_center_of_gravity(&mut self, cog: Pnt2){
        unsafe { ffi::cpBodySetCenterOfGravity(self.mut_ptr(), cog); }
    }
}

pub struct Arbiters<'a> {
    body: *const ffi::cpBody,
    node: *const ffi::cpArbiter,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for Arbiters<'a> {
    type Item = Arbiter<'a>;
    fn next(&mut self) -> Option<Arbiter<'a>> {
        unsafe {
            let ret = if !self.node.is_null() {
                let (mut a, mut b) = mem::uninitialized();
                ffi::cpArbiterGetBodies_NOSWAP(self.node, &mut a, &mut b, 0);
                Arbiter::from_ptr(self.node, a as *const _ == self.body)
            } else {
                return None;
            };
            self.node = ffi::cpBodyNextArbiter(self.body as *mut _, self.node as *mut _);
            Some(ret)
        }
    }
}

pub struct MutArbiters<'a>(Arbiters<'a>);

impl<'a> Iterator for MutArbiters<'a> {
    type Item = MutArbiter<'a>;
    fn next(&mut self) -> Option<MutArbiter<'a>> {
        unsafe { mem::transmute(self.0.next()) }
    }
}

pub struct Constraints<'a> {
    body: *const ffi::cpBody,
    node: *const ffi::cpConstraint,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for Constraints<'a> {
    type Item = &'a ConstraintBase;
    fn next(&mut self) -> Option<&'a ConstraintBase> {
        unsafe {
            if !self.node.is_null() {
                self.node = ffi::cpBodyNextConstraint(self.node as *mut _, self.body as *mut _);
                Some(ConstraintBase::from_ptr(self.node))
            } else {
                None
            }
        }
    }
}

pub struct MutConstraints<'a>(Constraints<'a>);

impl<'a> Iterator for MutConstraints<'a> {
    type Item = &'a mut ConstraintBase;
    fn next(&mut self) -> Option<&'a mut ConstraintBase> {
        unsafe { mem::transmute(self.0.next()) }
    }
}

pub struct Shapes<'a> {
    node: *const ffi::cpShape,
    _phantom: PhantomData<&'a ()>,
}

impl<'a> Iterator for Shapes<'a> {
    type Item = &'a ShapeBase;
    fn next(&mut self) -> Option<&'a ShapeBase> {
        unsafe {
            if !self.node.is_null() {
                self.node = ffi::cpBodyNextShape(self.node as *mut _);
                Some(ShapeBase::from_ptr(self.node))
            } else {
                None
            }
        }
    }
}

pub struct MutShapes<'a>(Shapes<'a>);

impl<'a> Iterator for MutShapes<'a> {
    type Item = &'a mut ShapeBase;
    fn next(&mut self) -> Option<&'a mut ShapeBase> {
        unsafe { mem::transmute(self.0.next()) }
    }
}
