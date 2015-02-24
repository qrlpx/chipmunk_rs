use super::{WILDCARD_COLLISION_TYPE};
use super::{Scalar, Pnt2, Vec2, CollisionType, ObjectHandle};
use bb::BB;
use arbiter::{ContactPointSet, Arbiter, MutArbiter};
use body::{BodyHandle, BodyBase};
use shape::{
    PointQueryInfo, SegmentQueryInfo, ShapeFilter, 
    ShapeHandle, ShapeBase,
};
use constraint::{ConstraintHandle, ConstraintBase};
use {ffi, na};

use libc::{c_int, c_void};

use std::collections::hash_set::{self, HashSet};
use std::collections::HashMap;
use std::ops::{Deref, DerefMut};
use std::{mem, ptr, raw};

/// Well, Chipmunk2D IS a C-lib, and here it shows. To adhere Rusts borrowing-principles, 
/// while still offering the same kind of flexibility Chipmunk2D does, we need to ensure
/// that Space and Arbiter are not borrowed mutably at the same time. Between only providing
/// ObjectHandles in Arbiter or crippling CollisionHandler, I've found this to be the most
/// intuitive solution.
pub struct SpaceAndArbiter<'a>(&'a mut LockedSpace, MutArbiter<'a>);

impl<'a> SpaceAndArbiter<'a> {
    pub fn space(&self) -> &LockedSpace { self.0 }
    pub fn arbiter(&self) -> &Arbiter<'a> { &*self.1 }
    pub fn mut_space(&mut self) -> &mut LockedSpace { self.0 }
    pub fn mut_arbiter(&mut self) -> &mut MutArbiter<'a> { &mut self.1 }
    
    unsafe fn new(space: *mut ffi::cpSpace, arb: *mut ffi::cpArbiter) 
        -> SpaceAndArbiter<'static>
    {
        SpaceAndArbiter(
            &mut**Space::from_mut_ptr(space),
            MutArbiter::from_mut_ptr(arb, (*arb).swapped == 1)
        )
    }
}

pub struct PostStepCallbacks(*mut ffi::cpSpace);

impl PostStepCallbacks {
    pub fn add<CB>(&mut self, cb: CB)
        where CB: FnOnce(&mut Space) + 'static
    {
        unsafe extern "C" fn func<CB>(space: *mut ffi::cpSpace,
                                      _: *mut c_void, 
                                      data: *mut c_void)
            where CB: FnOnce(&mut Space) + 'static
        {
            let mut cb: Box<CB> = mem::transmute(data);
            cb(Space::from_mut_ptr(space))
        }
        unsafe {
            ffi::cpSpaceAddPostStepCallback_NOKEYS(
                self.0, mem::transmute(func::<CB>), 
                ptr::null_mut(), mem::transmute(box cb)
            );
        }
    }
}

pub trait CollisionHandler: Sync + 'static {
    fn begin<'a>(&'a mut self, mut args: SpaceAndArbiter<'a>) -> bool {
        unsafe {
            let (arb, space) = (args.mut_arbiter().mut_ptr(), args.mut_space().mut_ptr());
            let ret_a = ffi::cpArbiterCallWildcardBeginA(arb, space);
            let ret_b = ffi::cpArbiterCallWildcardBeginB(arb, space);
            ret_a == 1 && ret_b == 1
        }
    } 
    fn pre_solve<'a>(&'a mut self, mut args: SpaceAndArbiter<'a>) -> bool { 
        unsafe {
            let (arb, space) = (args.mut_arbiter().mut_ptr(), args.mut_space().mut_ptr());
            let ret_a = ffi::cpArbiterCallWildcardPreSolveA(arb, space);
            let ret_b = ffi::cpArbiterCallWildcardPreSolveB(arb, space);
            ret_a == 1 && ret_b == 1
        }
    }
    fn post_solve<'a>(&'a mut self, mut args: SpaceAndArbiter<'a>){
        unsafe {
            let (arb, space) = (args.mut_arbiter().mut_ptr(), args.mut_space().mut_ptr());
            ffi::cpArbiterCallWildcardPostSolveA(arb, space);
            ffi::cpArbiterCallWildcardPostSolveB(arb, space);
        }
    }
    fn separate<'a>(&'a mut self, mut args: SpaceAndArbiter<'a>){
        unsafe {
            let (arb, space) = (args.mut_arbiter().mut_ptr(), args.mut_space().mut_ptr());
            ffi::cpArbiterCallWildcardSeparateA(arb, space);
            ffi::cpArbiterCallWildcardSeparateB(arb, space);
        }
    }
}

pub trait WildcardCollisionHandler: CollisionHandler {}

impl<H: WildcardCollisionHandler> CollisionHandler for H {
    fn begin<'a>(&'a mut self, mut args: SpaceAndArbiter<'a>) -> bool { true } 
    fn pre_solve<'a>(&'a mut self, mut args: SpaceAndArbiter<'a>) -> bool { true }
    fn post_solve<'a>(&'a mut self, mut args: SpaceAndArbiter<'a>){}
    fn separate<'a>(&'a mut self, mut args: SpaceAndArbiter<'a>){}
}

unsafe fn load_collision_handler<H>(handler: &Box<H>, 
                                    dest: *mut ffi::cpCollisionHandler)
    where H: CollisionHandler 
{
    unsafe extern "C" fn begin_func<H>(arb: *mut ffi::cpArbiter, 
                                       space: *mut ffi::cpSpace, 
                                       data: *mut c_void)
        -> ffi::cpBool
        where H: CollisionHandler
    {
        let handler: &mut H = mem::transmute(data);
        let ret = handler.begin(SpaceAndArbiter::new(space, arb));
        if ret { 1 } else { 0 }
    }
    (*dest).beginFunc = mem::transmute(begin_func::<H>);

    unsafe extern "C" fn pre_solve_func<H>(arb: *mut ffi::cpArbiter, 
                                           space: *mut ffi::cpSpace, 
                                           data: *mut c_void)
        -> ffi::cpBool
        where H: CollisionHandler
    {
        let handler: &mut H = mem::transmute(data);
        let ret = handler.pre_solve(SpaceAndArbiter::new(space, arb));
        if ret { 1 } else { 0 }
    }
    (*dest).preSolveFunc = mem::transmute(pre_solve_func::<H>);

    unsafe extern "C" fn post_solve_func<H>(arb: *mut ffi::cpArbiter, 
                                            space: *mut ffi::cpSpace, 
                                            data: *mut c_void)
        where H: CollisionHandler
    {
        let handler: &mut H = mem::transmute(data);
        handler.post_solve(SpaceAndArbiter::new(space, arb));
    }
    (*dest).postSolveFunc = mem::transmute(post_solve_func::<H>);

    unsafe extern "C" fn separate_func<H>(arb: *mut ffi::cpArbiter, 
                                          space: *mut ffi::cpSpace, 
                                          data: *mut c_void)
        where H: CollisionHandler
    {
        let handler: &mut H = mem::transmute(data);
        handler.separate(SpaceAndArbiter::new(space, arb));
    }
    (*dest).separateFunc = mem::transmute(separate_func::<H>);

    (*dest).userData = mem::transmute(&**handler);
}

pub struct LockedSpace {
    bodies: HashSet<BodyHandle>,
    constraints: HashSet<ConstraintHandle>,
    shapes: HashSet<ShapeHandle>,
    handlers: HashMap<(CollisionType, CollisionType), Box<CollisionHandler>>,
    raw: ffi::cpSpace,
}

unsafe impl Sync for LockedSpace {}

impl LockedSpace {
    pub fn ptr(&self) -> *const ffi::cpSpace { &self.raw }
    pub fn mut_ptr(&mut self) -> *mut ffi::cpSpace { self.ptr() as *mut _ }

    pub fn contains_body(&self, body: BodyHandle) -> bool {
        self.bodies.contains(&body) || body == self.ground_body().handle()
    }
    pub fn contains_constraint(&self, constraint: ConstraintHandle) -> bool {
        self.constraints.contains(&constraint)
    }
    pub fn contains_shape(&self, shape: ShapeHandle) -> bool {
        self.shapes.contains(&shape)
    }

    pub fn body(&self, body: BodyHandle) -> &BodyBase {
        assert!(self.contains_body(body));
        unsafe { BodyBase::from_ptr(body.unwrap()) }
    }
    pub fn mut_body(&mut self, body: BodyHandle) -> &mut BodyBase {
        unsafe { mem::transmute(self.body(body)) }
    }

    pub fn constraint(&self, constraint: ConstraintHandle) -> &ConstraintBase {
        assert!(self.contains_constraint(constraint));
        unsafe { ConstraintBase::from_ptr(constraint.unwrap()) }
    }
    pub fn mut_constraint(&mut self, constraint: ConstraintHandle) -> &mut ConstraintBase {
        unsafe { mem::transmute(self.constraint(constraint)) }
    }

    pub fn shape(&self, shape: ShapeHandle) -> &ShapeBase {
        assert!(self.contains_shape(shape));
        unsafe { ShapeBase::from_ptr(shape.unwrap()) }
    }
    pub fn mut_shape(&mut self, shape: ShapeHandle) -> &mut ShapeBase {
        unsafe { mem::transmute(self.shape(shape)) }
    }

    /// Number of iterations to use in the impulse solver to solve contacts and other constraints.
    pub fn iterations(&self) -> i32 {
        unsafe { ffi::cpSpaceGetIterations(self.ptr()) }
    }
    pub fn set_iterations(&mut self, iterations: i32){
        unsafe { ffi::cpSpaceSetIterations(self.mut_ptr(), iterations as c_int); }
    }

    /// Gravity to pass to rigid bodies when integrating velocity.
    pub fn gravity(&self) -> Vec2 {
        unsafe { ffi::cpSpaceGetGravity(self.ptr()).to_vec() }
    }
    pub fn set_gravity(&mut self, v: Vec2){
        unsafe { ffi::cpSpaceSetGravity(self.mut_ptr(), na::orig::<Pnt2>() + v); }
    }

    /// Damping rate expressed as the fraction of velocity bodies retain each second.
    /// A value of 0.9 would mean that each body's velocity will drop 10% per second.
    /// The default value is 1.0, meaning no damping is applied.
    /// 
    /// NOTE: This damping value is different than those of cpDampedSpring and 
    /// cpDampedRotarySpring.
    pub fn damping(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetDamping(self.ptr()) }
    }
    pub fn set_damping(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetDamping(self.mut_ptr(), v); }
    }

    /// Speed threshold for a body to be considered idle.
    /// The default value of 0 means to let the space guess a good threshold based on gravity.
    pub fn idle_speed_threshold(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetIdleSpeedThreshold(self.ptr()) }
    }
    pub fn set_idle_speed_threshold(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetIdleSpeedThreshold(self.mut_ptr(), v); }
    }
    
    /// Time a group of bodies must remain idle in order to fall asleep.
    /// Enabling sleeping also implicitly enables the the contact graph.
    /// The default value of INFINITY disables the sleeping algorithm.
    pub fn sleep_time_treshold(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetSleepTimeThreshold(self.ptr()) }
    }
    pub fn set_sleep_time_treshold(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetSleepTimeThreshold(self.mut_ptr(), v); }
    }
    
    /// Amount of encouraged penetration between colliding shapes.
    /// Used to reduce oscillating contacts and keep the collision cache warm.
    /// Defaults to 0.1. If you have poor simulation quality,
    /// increase this number as much as possible without allowing visible amounts of overlap.
    pub fn collision_slop(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetCollisionSlop(self.ptr()) }
    }
    pub fn set_collision_slot(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetCollisionSlop(self.mut_ptr(), v); }
    }
    
    /// Determines how fast overlapping shapes are pushed apart.
    /// Expressed as a fraction of the error remaining after each second.
    /// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap 
    /// each frame at 60Hz.
    pub fn collision_bias(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetCollisionBias(self.ptr()) }
    }
    pub fn set_collision_bias(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetCollisionBias(self.mut_ptr(), v); }
    }
    
    /// Number of frames that contact information should persist.
    /// Defaults to 3. There is probably never a reason to change this value.
    pub fn collision_persistence(&self) -> u32 {
        unsafe { ffi::cpSpaceGetCollisionPersistence(self.ptr()) }
    }
    pub fn set_collision_persistence(&mut self, v: u32){
        unsafe { ffi::cpSpaceSetCollisionPersistence(self.mut_ptr(), v) }
    }
    
    /// TODO Userdata leak?
    /// The Space provided static body for a given cpSpace.
    /// This is merely provided for convenience and you are not required to use it.
    pub fn ground_body(&self) -> &BodyBase {
        unsafe {
            let ptr = ffi::cpSpaceGetStaticBody(self.ptr() as *mut _);
            BodyBase::from_ptr(ptr)
        }
    }
    pub fn mut_ground_body(&mut self) -> &mut BodyBase {
        unsafe { mem::transmute(self.ground_body()) }
    }

    /// Returns the current (or most recent) time step used with the given space.
    /// Useful from callbacks if your time step is not a compile-time global.
    pub fn current_timestep(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetCurrentTimeStep(self.ptr()) }
    }


    /// Query the space at a point and call @c func for each shape found.
    ///
    /// TODO: mut-variant. should this return bool?
    pub fn point_query<CB>(&self, point: Pnt2, max_dist: Scalar, filter: ShapeFilter, cb: CB)
        where CB: FnMut(&ShapeBase, PointQueryInfo)
    {
        unsafe extern "C" fn func<CB>(
            shape: *mut ffi::cpShape, point: Pnt2, dist: Scalar, gradient: Pnt2, 
            data: *mut c_void)
            where CB: FnMut(&ShapeBase, PointQueryInfo)
        {
            let closure: &mut CB = mem::transmute(data);
            let info = PointQueryInfo::new(point, dist, gradient.to_vec());
            closure(ShapeBase::from_ptr(shape), info);
        }
    
        unsafe {
            ffi::cpSpacePointQuery_NOLOCK(
                self.ptr() as *mut _, point, max_dist, filter,
                mem::transmute(func::<CB>), mem::transmute(&cb)
            );
        }
    }

    /// Query the space at a point and return the nearest shape found. 
    pub fn point_query_nearest(&self, point: Pnt2, max_dist: Scalar, filter: ShapeFilter)
        -> Option<(&ShapeBase, PointQueryInfo)>
    {
        unsafe {
            let mut info = mem::uninitialized();
            let shape = ffi::cpSpacePointQueryNearest(
                self.ptr() as *mut _, point, max_dist, filter, &mut info
            );
            if !shape.is_null() {
                Some((
                    ShapeBase::from_ptr(shape),
                    PointQueryInfo::new(info.point, info.distance, info.gradient.to_vec())
                ))
            } else {
                None
            }
        }
    }

    /// Perform a directed line segment query (like a raycast) against the space 
    /// calling @c func for each shape intersected.
    ///
    /// TODO: mut-variant. should this return bool?
    pub fn segment_query<CB>(&self, 
                             start: Pnt2, 
                             end: Pnt2, 
                             thickness: Scalar,
                             filter: ShapeFilter, 
                             cb: CB)
        where CB: FnMut(&ShapeBase, SegmentQueryInfo)
    {
        unsafe extern "C" fn func<CB>(
            shape: *mut ffi::cpShape, point: Pnt2, normal: Pnt2, alpha: Scalar, 
            data: *mut c_void)
            where CB: FnMut(&ShapeBase, SegmentQueryInfo)
        {
            let closure: &mut CB = mem::transmute(data);
            let info = SegmentQueryInfo::new(point, normal.to_vec(), alpha);
            closure(ShapeBase::from_ptr(shape), info);
        }

        unsafe {
            ffi::cpSpaceSegmentQuery_NOLOCK(
                self.ptr() as *mut _, start, end, thickness, filter,
                mem::transmute(func::<CB>), mem::transmute(&cb)
            );
        }
    }

    /// Perform a directed line segment query (like a raycast) against the space 
    /// and return the first shape hit. 
    pub fn segment_query_first(&self, 
                               start: Pnt2, 
                               end: Pnt2, 
                               thickness: Scalar,
                               filter: ShapeFilter)
        -> Option<(&ShapeBase,  SegmentQueryInfo)> 
    {
        unsafe {
            let mut info = mem::uninitialized();
            let shape = ffi::cpSpaceSegmentQueryFirst(
                self.ptr() as *mut _, start, end, thickness, filter, &mut info
            );
            if !shape.is_null() {
                Some((
                    ShapeBase::from_ptr(shape),
                    SegmentQueryInfo::new(info.point, info.normal.to_vec(), info.alpha)
                ))
            } else {
                None
            }
        }
    }


    /// Perform a fast rectangle query on the space calling @c func for each shape found.
    /// Only the shape's bounding boxes are checked for overlap, not their full shape.
    ///
    /// TODO: mut-variant. should this return bool?
    pub fn bb_query<CB>(&self, bb: BB, filter: ShapeFilter, cb: CB)
        where CB: FnMut(&ShapeBase)
    {
        unsafe extern "C" fn func<CB>(shape: *mut ffi::cpShape, data: *mut c_void)
            where CB: FnMut(&ShapeBase)
        {
            let closure: &mut CB = mem::transmute(data);
            closure(ShapeBase::from_ptr(shape));
        }

        unsafe {
            ffi::cpSpaceBBQuery_NOLOCK(
                self.ptr() as *mut _, bb, filter,
                mem::transmute(func::<CB>), mem::transmute(&cb)
            );
        }
    }

/* 
/// Shape query callback function type.
typedef void (*cpSpaceShapeQueryFunc)(cpShape *shape, cpContactPointSet *points, void *data);
/// Query a space for any shapes overlapping the given shape and call @c func for each shape found.
TODO cpBool cpSpaceShapeQuery(cpSpace *space, cpShape *shape, cpSpaceShapeQueryFunc func, void *data);

*/

    pub fn bodies(&self) -> Bodies {
        Bodies{ iter: self.bodies.iter() }
    }
    pub fn constraints(&self) -> Constraints {
        Constraints{ iter: self.constraints.iter() }
    }
    pub fn shapes(&self) -> Shapes {
        Shapes{ iter: self.shapes.iter() }
    }

    pub fn mut_bodies(&mut self) -> MutBodies {
        MutBodies(self.bodies())
    }
    pub fn mut_constraints(&mut self) -> MutConstraints {
        MutConstraints(self.constraints())
    }
    pub fn mut_shapes(&mut self) -> MutShapes {
        MutShapes(self.shapes())
    }

    /// Update the collision detection info for the static shapes in the space.
    pub fn reindex_static(&mut self){
        unsafe { ffi::cpSpaceReindexStatic(self.mut_ptr()); }
    }
    /// Update the collision detection data for a specific shape in the space.
    pub fn reindex_shape(&mut self, shape: ShapeHandle){
        assert!(self.contains_shape(shape));
        unsafe { 
            ffi::cpSpaceReindexShape(
                self.mut_ptr(), shape.unwrap()
            ); 
        }
    }
    /// Update the collision detection data for all shapes attached to a body.
    pub fn reindex_shapes_for_body(&mut self, body: BodyHandle){
        assert!(self.contains_body(body));
        unsafe { 
            ffi::cpSpaceReindexShapesForBody(
                self.mut_ptr(), body.unwrap()
            ); 
        } 
    }
}

pub struct Space(LockedSpace);

impl Deref for Space {
    type Target = LockedSpace;
    fn deref<'a>(&'a self) -> &'a LockedSpace { &self.0 }
}
impl DerefMut for Space {
    fn deref_mut<'a>(&'a mut self) -> &'a mut LockedSpace { &mut self.0 }
}

impl Space {
    /// This should only be called from inside a CollisionHandler or PostStepCallback.
    pub unsafe fn from_ptr(ptr: *const ffi::cpSpace) -> &'static Self {
        mem::transmute(ffi::cpSpaceGetUserData(ptr))
    }
    pub unsafe fn from_mut_ptr(ptr: *mut ffi::cpSpace) -> &'static mut Self {
        mem::transmute(Space::from_ptr(ptr))
    }

    pub fn new() -> Space {
        Space(LockedSpace {
            bodies: HashSet::new(),
            constraints: HashSet::new(),
            shapes: HashSet::new(),
            handlers: HashMap::new(),
            raw: unsafe { 
                let mut data = mem::uninitialized();
                ffi::cpSpaceInit(&mut data);
                data    
            }
        })
    }

    pub fn add_body(&mut self, mut body: Box<BodyBase>) -> BodyHandle {
        let handle = body.handle();
        unsafe {
            mem::forget(body);
            assert!(
                ffi::cpBodyGetSpace(handle.unwrap()).is_null(), 
                "Body is already added to a Space!"
            );
            ffi::cpSpaceAddBody(self.mut_ptr(), handle.unwrap());
        }
        self.bodies.insert(handle);
        handle
    }

    
    pub unsafe fn add_constraint(&mut self, mut constraint: Box<ConstraintBase>) 
        -> ConstraintHandle
    {
        let handle = constraint.handle();
        mem::forget(constraint);
        assert!(
            ffi::cpConstraintGetSpace(handle.unwrap()).is_null(), 
            "Constraint is already added to a Space!"
        );
        ffi::cpSpaceAddConstraint(self.mut_ptr(), handle.unwrap());
        self.constraints.insert(handle);
        handle
    }
    pub fn attach_constraint(&mut self, 
                             bodies: (BodyHandle, BodyHandle), 
                             mut constraint: Box<ConstraintBase>)
        -> ConstraintHandle
    {
        unsafe {
            assert!(self.contains_body(bodies.0));
            assert!(self.contains_body(bodies.1));
            assert!(!constraint.is_attached(), "Constraint is already attached to Bodies!");
            constraint.set_bodies(Some(bodies));
            self.add_constraint(constraint)
        }
    }

    pub unsafe fn add_shape(&mut self, mut shape: Box<ShapeBase>) 
        -> ShapeHandle
    {
        let handle = shape.handle();
        mem::forget(shape);
        assert!(
            ffi::cpShapeGetSpace(handle.unwrap()).is_null(), 
            "Shape is already added to a Space!"
        );
        ffi::cpSpaceAddShape(self.mut_ptr(), handle.unwrap());
        self.shapes.insert(handle);
        handle
    }
    pub fn attach_shape(&mut self, body: BodyHandle, mut shape: Box<ShapeBase>) 
        -> ShapeHandle
    {
        unsafe {
            assert!(self.contains_body(body));
            assert!(!shape.is_attached(), "Shape is already attached to a Body!");
            shape.set_body(Some(body));
            self.add_shape(shape)
        }
    }

   pub fn remove_body(&mut self, handle: BodyHandle) -> Box<BodyBase> {
        if self.bodies.remove(&handle) {
            let mut ret = unsafe { BodyBase::box_from_ptr(handle.unwrap()) };
            for constraint in ret.constraints().collect::<Vec<_>>().iter() {
                self.remove_constraint(constraint.handle());
            }
            for shape in ret.shapes().collect::<Vec<_>>().iter() {
                self.remove_shape(shape.handle());
            }
            ret
        } else {
            panic!("Space doesn't contain Body!")
        }
    }
    pub fn remove_constraint(&mut self, handle: ConstraintHandle) -> Box<ConstraintBase> {
        if self.constraints.remove(&handle) {
            let mut ret = unsafe { ConstraintBase::box_from_ptr(handle.unwrap()) };
            unsafe { ret.set_bodies(None); }
            ret
        } else {
            panic!("Space doesn't contain Constraint!")
        }
    }
    pub fn remove_shape(&mut self, handle: ShapeHandle) -> Box<ShapeBase> {
        if self.shapes.remove(&handle) {
            let mut ret = unsafe { ShapeBase::box_from_ptr(handle.unwrap()) };
            unsafe { ret.set_body(None); }
            ret
        } else {
            panic!("Space doesn't contain Shape!")
        }
    }

    pub fn set_collision_handler<H>(&mut self, 
                                    (ty_a, ty_b): (CollisionType, CollisionType), 
                                    handler: Box<H>)                                                
        where H: CollisionHandler
    {
        assert!(ty_a != WILDCARD_COLLISION_TYPE);
        assert!(ty_b != WILDCARD_COLLISION_TYPE);
        unsafe { load_collision_handler::<H>(
            &handler, ffi::cpSpaceAddCollisionHandler(self.mut_ptr(), ty_a, ty_b)
        ); }
        self.handlers.insert((ty_a, ty_b), handler);
    }
    pub fn set_wildcard_collision_handler<H>(&mut self, ty: CollisionType, handler: Box<H>)
        where H: WildcardCollisionHandler
    {
        assert!(ty != WILDCARD_COLLISION_TYPE);
        unsafe { load_collision_handler::<H>(
            &handler, ffi::cpSpaceAddWildcardHandler(self.mut_ptr(), ty)
        ); }
        self.handlers.insert((WILDCARD_COLLISION_TYPE, ty), handler);
    }

    /// Switch the space to use a spatial hash as it's spatial index.
    pub fn use_spatial_hash(&mut self, dim: Scalar, count: i32){
        unsafe { ffi::cpSpaceUseSpatialHash(self.mut_ptr(), dim, count); }
    }

    /// Step the space forward in time by @c dt.
    pub fn step(&mut self, dt: Scalar){
        unsafe { 
            ffi::cpSpaceSetUserData(self.mut_ptr(), mem::transmute(&mut*self));
            ffi::cpSpaceStep(self.mut_ptr(), dt);
        }
    }
}

#[unsafe_destructor]
impl Drop for Space {
    /// TODO: ground body!
    fn drop(&mut self){
        unsafe {
            let bodies = self.bodies()
                .map(|body| body.handle())
                .collect::<Vec<BodyHandle>>();
            for handle in bodies {
                self.remove_body(handle);
            }
            ffi::cpSpaceDestroy(self.mut_ptr());
        }
    }
}

macro_rules! iterators {
    ($item:ident, $iter:ident, $mut_iter:ident, $handle:ident) => (

        pub struct $iter<'a> {
            iter: hash_set::Iter<'a, $handle>
        }

        impl<'a> Iterator for $iter<'a> {
            type Item = &'a $item;
            fn next(&mut self) -> Option<&'a $item> {
                self.iter.next().map(|handle|
                    unsafe { $item::from_ptr(handle.unwrap()) }
                )
            }
        }

        pub struct $mut_iter<'a>($iter<'a>);

        impl<'a> Iterator for $mut_iter<'a> {
            type Item = &'a mut $item;
            fn next(&mut self) -> Option<&'a mut $item> {
                unsafe { mem::transmute(self.0.next()) }
            }
        }                                    
                                            
    );
}

iterators!(BodyBase, Bodies, MutBodies, BodyHandle);
iterators!(ConstraintBase, Constraints, MutConstraints, ConstraintHandle);
iterators!(ShapeBase, Shapes, MutShapes, ShapeHandle);

