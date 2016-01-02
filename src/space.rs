use super::{WILDCARD_COLLISION_TYPE};
use super::{Scalar, Pnt2, Vec2, CollisionType};
use bb::BB;
use arbiter::{ContactPointSet, Arbiter, ArbiterMut};
use body::{BodyHandle, Body};
use shape::{
    PointQueryInfo, SegmentQueryInfo, ShapeFilter, 
    ShapeHandle, Shape,
};
use constraint::{ConstraintHandle, Constraint};
use {ffi, na};

use fnv::FnvHasher;
use libc::{c_int, c_void};

use std::collections::{hash_set, HashSet, HashMap};
use std::collections::hash_state::DefaultState;
use std::ops::{Deref, DerefMut};
use std::{mem, ptr, raw};

// ++++++++++++++++++++ CollisionHandler ++++++++++++++++++++

pub trait CollisionHandler: 'static {
    fn begin<'a>(&'a mut self, space: &mut LockedSpace, arbiter: ArbiterMut<'a>) -> bool { 
        true 
    } 

    fn pre_solve<'a>(&'a mut self, space: &mut LockedSpace, arbiter: ArbiterMut<'a>) -> bool { 
        true
    } 

    fn post_solve<'a>(&'a mut self, space: &mut LockedSpace, arbiter: Arbiter<'a>){ 
        /* */
    }

    fn separate<'a>(&'a mut self, space: &mut LockedSpace, arbiter: Arbiter<'a>){
        /* */ 
    }
}

unsafe fn load_collision_handler<H>(handler: &mut Box<H>, dest: *mut ffi::cpCollisionHandler)
    where H: CollisionHandler 
{
    extern "C" fn begin_func<H>(arb: *mut ffi::cpArbiter, 
                                space: *mut ffi::cpSpace, 
                                data: *mut c_void)
        -> ffi::cpBool
        where H: CollisionHandler
    {
        unsafe {
            let handler: &mut H = mem::transmute(data);
            let ret = handler.begin(
                LockedSpace::from_mut_ptr(space),
                ArbiterMut::from_mut_ptr(arb, (*arb).swapped == 1)
            );
            if ret { 1 } else { 0 }
        }
    }
    (*dest).beginFunc = Some(begin_func::<H>);

    extern "C" fn pre_solve_func<H>(arb: *mut ffi::cpArbiter, 
                                    space: *mut ffi::cpSpace, 
                                    data: *mut c_void)
        -> ffi::cpBool
        where H: CollisionHandler
    {
        unsafe {
            let handler: &mut H = mem::transmute(data);
            let ret = handler.pre_solve(
                LockedSpace::from_mut_ptr(space),
                ArbiterMut::from_mut_ptr(arb, (*arb).swapped == 1)
            );
            if ret { 1 } else { 0 }
        }
    }
    (*dest).preSolveFunc = Some(pre_solve_func::<H>);

    extern "C" fn post_solve_func<H>(arb: *mut ffi::cpArbiter, 
                                     space: *mut ffi::cpSpace, 
                                     data: *mut c_void)
        where H: CollisionHandler
    {
        unsafe {
            let handler: &mut H = mem::transmute(data);
            handler.post_solve(
                LockedSpace::from_mut_ptr(space),
                Arbiter::from_ptr(arb, (*arb).swapped == 1)
            );
        }
    }
    (*dest).postSolveFunc = Some(post_solve_func::<H>);

    extern "C" fn separate_func<H>(arb: *mut ffi::cpArbiter, 
                                   space: *mut ffi::cpSpace, 
                                   data: *mut c_void)
        where H: CollisionHandler
    {
        unsafe {
            let handler: &mut H = mem::transmute(data);
            handler.separate(
                LockedSpace::from_mut_ptr(space),
                Arbiter::from_ptr(arb, (*arb).swapped == 1)
            );
        }
    }
    (*dest).separateFunc = Some(separate_func::<H>);

    (*dest).userData = mem::transmute(&mut**handler);
}

// ++++++++++++++++++++ Space ++++++++++++++++++++

pub struct LockedSpace {
    data: ffi::cpSpace,
    bodies: HashSet<BodyHandle, DefaultState<FnvHasher>>,
    constraints: HashSet<ConstraintHandle, DefaultState<FnvHasher>>,
    shapes: HashSet<ShapeHandle, DefaultState<FnvHasher>>,
    handlers: HashMap<(CollisionType, CollisionType), Box<CollisionHandler>>,
}

unsafe impl Sync for LockedSpace {}

impl LockedSpace {
    #[doc(hidden)]
    pub unsafe fn from_ptr(ptr: *const ffi::cpSpace) -> &'static Self {
        mem::transmute(ptr)
    }
    #[doc(hidden)]
    pub unsafe fn from_mut_ptr(ptr: *mut ffi::cpSpace) -> &'static mut Self {
        mem::transmute(ptr)
    }
    #[doc(hidden)]
    pub fn as_ptr(&self) -> *const ffi::cpSpace { &self.data }
    #[doc(hidden)]
    pub fn as_mut_ptr(&mut self) -> *mut ffi::cpSpace { &mut self.data }

    pub fn contains_body(&self, body: BodyHandle) -> bool {
        self.bodies.contains(&body) //|| body == self.ground_body().handle() TODO
    }
    pub fn contains_constraint(&self, constraint: ConstraintHandle) -> bool {
        self.constraints.contains(&constraint)
    }
    pub fn contains_shape(&self, shape: ShapeHandle) -> bool {
        self.shapes.contains(&shape)
    }

    pub fn body(&self, body: BodyHandle) -> &Body {
        assert!(self.contains_body(body));
        unsafe { Body::from_ptr(body.as_ptr()) }
    }
    pub fn body_mut(&mut self, body: BodyHandle) -> &mut Body {
        assert!(self.contains_body(body));
        unsafe { Body::from_mut_ptr(body.as_mut_ptr()) }
    }

    pub fn constraint(&self, constraint: ConstraintHandle) -> &Constraint {
        assert!(self.contains_constraint(constraint));
        unsafe { Constraint::from_ptr(constraint.as_ptr()) }
    }
    pub fn constraint_mut(&mut self, constraint: ConstraintHandle) -> &mut Constraint {
        assert!(self.contains_constraint(constraint));
        unsafe { Constraint::from_mut_ptr(constraint.as_mut_ptr()) }
    }

    pub fn shape(&self, shape: ShapeHandle) -> &Shape {
        assert!(self.contains_shape(shape));
        unsafe { Shape::from_ptr(shape.as_ptr()) }
    }
    pub fn shape_mut(&mut self, shape: ShapeHandle) -> &mut Shape {
        assert!(self.contains_shape(shape));
        unsafe { Shape::from_mut_ptr(shape.as_mut_ptr()) }
    }

    /// Number of iterations to use in the impulse solver to solve contacts and other constraints.
    pub fn iterations(&self) -> i32 {
        unsafe { ffi::cpSpaceGetIterations(self.as_ptr()) }
    }
    pub fn set_iterations(&mut self, iterations: i32){
        unsafe { ffi::cpSpaceSetIterations(self.as_mut_ptr(), iterations as c_int); }
    }

    /// Gravity to pass to rigid bodies when integrating velocity.
    pub fn gravity(&self) -> Vec2 {
        unsafe { ffi::cpSpaceGetGravity(self.as_ptr()) }
    }
    pub fn set_gravity(&mut self, v: Vec2){
        unsafe { ffi::cpSpaceSetGravity(self.as_mut_ptr(), v); }
    }

    /// Damping rate expressed as the fraction of velocity bodies retain each second.
    /// A value of 0.9 would mean that each body's velocity will drop 10% per second.
    /// The default value is 1.0, meaning no damping is applied.
    /// 
    /// NOTE: This damping value is different than those of cpDampedSpring and 
    /// cpDampedRotarySpring.
    pub fn damping(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetDamping(self.as_ptr()) }
    }
    pub fn set_damping(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetDamping(self.as_mut_ptr(), v); }
    }

    /// Speed threshold for a body to be considered idle.
    /// The default value of 0 means to let the space guess a good threshold based on gravity.
    pub fn idle_speed_threshold(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetIdleSpeedThreshold(self.as_ptr()) }
    }
    pub fn set_idle_speed_threshold(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetIdleSpeedThreshold(self.as_mut_ptr(), v); }
    }
    
    /// Time a group of bodies must remain idle in order to fall asleep.
    /// Enabling sleeping also implicitly enables the the contact graph.
    /// The default value of INFINITY disables the sleeping algorithm.
    pub fn sleep_time_treshold(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetSleepTimeThreshold(self.as_ptr()) }
    }
    pub fn set_sleep_time_treshold(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetSleepTimeThreshold(self.as_mut_ptr(), v); }
    }
    
    /// Amount of encouraged penetration between colliding shapes.
    /// Used to reduce oscillating contacts and keep the collision cache warm.
    /// Defaults to 0.1. If you have poor simulation quality,
    /// increase this number as much as possible without allowing visible amounts of overlap.
    pub fn collision_slop(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetCollisionSlop(self.as_ptr()) }
    }
    pub fn set_collision_slot(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetCollisionSlop(self.as_mut_ptr(), v); }
    }
    
    /// Determines how fast overlapping shapes are pushed apart.
    /// Expressed as a fraction of the error remaining after each second.
    /// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap 
    /// each frame at 60Hz.
    pub fn collision_bias(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetCollisionBias(self.as_ptr()) }
    }
    pub fn set_collision_bias(&mut self, v: Scalar){
        unsafe { ffi::cpSpaceSetCollisionBias(self.as_mut_ptr(), v); }
    }
    
    /// Number of frames that contact information should persist.
    /// Defaults to 3. There is probably never a reason to change this value.
    pub fn collision_persistence(&self) -> u32 {
        unsafe { ffi::cpSpaceGetCollisionPersistence(self.as_ptr()) }
    }
    pub fn set_collision_persistence(&mut self, v: u32){
        unsafe { ffi::cpSpaceSetCollisionPersistence(self.as_mut_ptr(), v) }
    }

    /* TODO any need for this?
    /// The Space provided static body for a given cpSpace.
    /// This is merely provided for convenience and you are not required to use it.
    pub fn ground_body(&self) -> &Body {
        unsafe {
            let ptr = ffi::cpSpaceGetStaticBody(self.as_ptr() as *mut _);
            Body::from_ptr(ptr)
        }
    }
    pub fn mut_ground_body(&mut self) -> &mut Body {
        unsafe { mem::transmute(self.ground_body()) }
    }
    */

    /// Returns the current (or most recent) time step used with the given space.
    /// Useful from callbacks if your time step is not a compile-time global.
    pub fn current_timestep(&self) -> Scalar {
        unsafe { ffi::cpSpaceGetCurrentTimeStep(self.as_ptr()) }
    }

    /// Query the space at a point and call @c func for each shape found.
    ///
    /// TODO: mut-variant. should this return bool?
    pub fn point_query<CB>(&self, 
                           point: Pnt2,
                           max_dist: Scalar, 
                           filter: ShapeFilter, 
                           cb: &mut CB)
        where CB: FnMut(&Shape, PointQueryInfo)
    {
        extern "C" fn func<CB>(shape: *mut ffi::cpShape, 
                               point: Vec2,
                               dist: Scalar, 
                               gradient: Vec2, 
                               data: *mut c_void)
            where CB: FnMut(&Shape, PointQueryInfo)
        {
            unsafe {
                let closure: &mut CB = mem::transmute(data);
                let info = PointQueryInfo::new(point.to_pnt(), dist, gradient);
                closure(Shape::from_ptr(shape), info);
            }
        }
    
        unsafe { 
            ffi::cpSpacePointQuery_NOLOCK(
                self.as_ptr() as *mut _,
                point.to_vec(), max_dist, filter, 
                Some(func::<CB>), cb as *mut CB as *mut c_void
            );
        }
    }

    /// Query the space at a point and return the nearest shape found. 
    pub fn point_query_nearest(&self, 
                               point: Pnt2,
                               max_dist: Scalar,
                               filter: ShapeFilter) 
                               -> Option<(&Shape, PointQueryInfo)> 
    {
        unsafe {
            let mut info = mem::zeroed();
            let shape = ffi::cpSpacePointQueryNearest(
                self.as_ptr() as *mut _, point.to_vec(), max_dist, filter, &mut info
            );
            if !shape.is_null() {
                Some((
                    Shape::from_ptr(shape),
                    PointQueryInfo::new(info.point.to_pnt(), info.distance, info.gradient)
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
                             cb: &mut CB)
        where CB: FnMut(&Shape, SegmentQueryInfo)
    {
        extern "C" fn func<CB>(shape: *mut ffi::cpShape, 
                                      point: Vec2, 
                                      normal: Vec2, 
                                      alpha: Scalar, 
                                      data: *mut c_void)
            where CB: FnMut(&Shape, SegmentQueryInfo)
        {
            unsafe {
                let closure: &mut CB = mem::transmute(data);
                let info = SegmentQueryInfo::new(point.to_pnt(), normal, alpha);
                closure(Shape::from_ptr(shape), info);
            }
        }

        unsafe {
            ffi::cpSpaceSegmentQuery_NOLOCK(
                self.as_ptr() as *mut _, 
                start.to_vec(), end.to_vec(), thickness, filter, 
                Some(func::<CB>), cb as *mut CB as *mut c_void
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
                               -> Option<(&Shape, SegmentQueryInfo)> 
    {
        unsafe {
            let mut info = mem::zeroed();
            let shape = ffi::cpSpaceSegmentQueryFirst(
                self.as_ptr() as *mut _, 
                start.to_vec(), end.to_vec(), thickness, filter, 
                &mut info
            );
            if !shape.is_null() {
                Some((
                    Shape::from_ptr(shape),
                    SegmentQueryInfo::new(info.point.to_pnt(), info.normal, info.alpha)
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
    pub fn bb_query<CB>(&self, bb: BB, filter: ShapeFilter, cb: &mut CB)
        where CB: FnMut(&Shape)
    {
        extern "C" fn func<CB>(shape: *mut ffi::cpShape, data: *mut c_void)
            where CB: FnMut(&Shape)
        {
            unsafe {
                let closure: &mut CB = mem::transmute(data);
                closure(Shape::from_ptr(shape));
            }
        }

        unsafe {
            ffi::cpSpaceBBQuery_NOLOCK(
                self.as_ptr() as *mut _, 
                bb, filter,
                Some(func::<CB>), cb as *mut CB as *mut c_void
            );
        }
    }
/* 
/// Shape query callback function type.
typedef void (*cpSpaceShapeQueryFunc)(cpShape *shape, cpContactPointSet *points, void *data);
/// Query a space for any shapes overlapping the given shape and call @c func for each shape found.
TODO cpBool cpSpaceShapeQuery(cpSpace *space, cpShape *shape, cpSpaceShapeQueryFunc func, void *data);
*/
    pub fn shapes(&self) -> Shapes {
        Shapes{ iter: self.shapes.iter() }
    }
    pub fn shapes_mut(&mut self) -> ShapesMut {
        ShapesMut{ iter: self.shapes.iter() }
    }

    pub fn bodies(&self) -> Bodies {
        Bodies{ iter: self.bodies.iter() }
    }
    pub fn bodies_mut(&mut self) -> BodiesMut {
        BodiesMut{ iter: self.bodies.iter() }
    }

    pub fn constraints(&self) -> Constraints {
        Constraints{ iter: self.constraints.iter() }
    }
    pub fn constraint_muts(&mut self) -> ConstraintsMut {
        ConstraintsMut{ iter: self.constraints.iter() }
    }

    /// Update the collision detection info for the static shapes in the space.
    pub fn reindex_static(&mut self){
        unsafe { ffi::cpSpaceReindexStatic(self.as_mut_ptr()); }
    }
    /// Update the collision detection data for a specific shape in the space.
    pub fn reindex_shape(&mut self, shape: ShapeHandle){
        assert!(self.contains_shape(shape));
        unsafe { ffi::cpSpaceReindexShape(self.as_mut_ptr(), shape.as_mut_ptr()); }
    }
    /// Update the collision detection data for all shapes attached to a body.
    pub fn reindex_shapes_for_body(&mut self, body: BodyHandle){
        assert!(self.contains_body(body));
        unsafe { ffi::cpSpaceReindexShapesForBody(self.as_mut_ptr(), body.as_mut_ptr()); } 
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
    pub fn new() -> Space {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpSpaceInit(&mut data);
            Space(LockedSpace {
                data: data,
                bodies: HashSet::default(),
                constraints: HashSet::default(),
                shapes: HashSet::default(),
                handlers: HashMap::new(),
            })
        }
    }

    pub fn add_body(&mut self, mut body: Box<Body>) -> BodyHandle {
        let handle = body.handle();
        unsafe {
            mem::forget(body);
            assert!(
                ffi::cpBodyGetSpace(handle.as_ptr()).is_null(), 
                "Body is already added to a Space!"
            );
            ffi::cpSpaceAddBody(self.as_mut_ptr(), handle.as_mut_ptr());
        }
        self.bodies.insert(handle);
        handle
    }
   
    pub unsafe fn add_constraint(&mut self, 
                                 mut constraint: Box<Constraint>) 
                                 -> ConstraintHandle 
    {
        let handle = constraint.handle();
        mem::forget(constraint);
        assert!(
            ffi::cpConstraintGetSpace(handle.as_ptr()).is_null(), 
            "Constraint is already added to a Space!"
        );
        ffi::cpSpaceAddConstraint(self.as_mut_ptr(), handle.as_mut_ptr());
        self.constraints.insert(handle);
        handle
    }

    pub fn attach_constraint(&mut self, 
                             bodies: (BodyHandle, BodyHandle), 
                             mut constraint: Box<Constraint>) 
                             -> ConstraintHandle {
        unsafe {
            assert!(self.contains_body(bodies.0));
            assert!(self.contains_body(bodies.1));
            assert!(!constraint.is_attached(), "Constraint is already attached to Bodies!");
            constraint.set_bodies(Some(bodies));
            self.add_constraint(constraint)
        }
    }

    pub unsafe fn add_shape(&mut self, mut shape: Box<Shape>) -> ShapeHandle {
        let handle = shape.handle();
        mem::forget(shape);
        assert!(
            ffi::cpShapeGetSpace(handle.as_ptr()).is_null(), 
            "Shape is already added to a Space!"
        );
        ffi::cpSpaceAddShape(self.as_mut_ptr(), handle.as_mut_ptr());
        self.shapes.insert(handle);
        handle
    }

    pub fn attach_shape(&mut self, body: BodyHandle, mut shape: Box<Shape>) -> ShapeHandle {
        unsafe {
            assert!(self.contains_body(body));
            assert!(!shape.is_attached(), "Shape is already attached to a Body!");
            shape.set_body(Some(body));
            self.add_shape(shape)
        }
    }

   pub fn remove_body(&mut self, handle: BodyHandle) -> Box<Body> {
        if self.bodies.remove(&handle) {
            let mut ret = unsafe { Body::from_owned_ptr(handle.as_mut_ptr()) };
            for constraint in ret.constraints() {
                self.remove_constraint(constraint);
            }
            for shape in ret.shapes() {
                self.remove_shape(shape);
            }
            unsafe { ffi::cpSpaceRemoveBody(self.as_mut_ptr(), ret.as_mut_ptr()); }
            ret
        } else {
            panic!("Space doesn't contain Body!")
        }
    }
    pub fn remove_constraint(&mut self, handle: ConstraintHandle) -> Box<Constraint> {
        if self.constraints.remove(&handle) {
            let mut ret = unsafe { Constraint::from_owned_ptr(handle.as_mut_ptr()) };
            unsafe { ffi::cpSpaceRemoveConstraint(self.as_mut_ptr(), ret.as_mut_ptr()); }
            unsafe { ret.set_bodies(None); }
            ret
        } else {
            panic!("Space doesn't contain Constraint!")
        }
    }
    pub fn remove_shape(&mut self, handle: ShapeHandle) -> Box<Shape> {
        if self.shapes.remove(&handle) {
            let mut ret = unsafe { Shape::from_owned_ptr(handle.as_mut_ptr()) };
            unsafe { ffi::cpSpaceRemoveShape(self.as_mut_ptr(), ret.as_mut_ptr()); }
            unsafe { ret.set_body(None); }
            ret
        } else {
            panic!("Space doesn't contain Shape!")
        }
    }

    pub fn set_collision_handler<H>(&mut self, 
                                    (ty_a, ty_b): (CollisionType, CollisionType), 
                                    mut handler: Box<H>)
        where H: CollisionHandler
    {
        unsafe { 
            load_collision_handler::<H>(
                &mut handler, ffi::cpSpaceAddCollisionHandler(self.as_mut_ptr(), ty_a, ty_b)
            ); 
        }
        self.handlers.insert((ty_a, ty_b), handler);
    }

    pub fn set_wildcard_collision_handler<H>(&mut self, ty: CollisionType, mut handler: Box<H>)
        where H: CollisionHandler
    {
        unsafe { 
            load_collision_handler::<H>(
                &mut handler, ffi::cpSpaceAddWildcardHandler(self.as_mut_ptr(), ty)
            );
        }
        self.handlers.insert((WILDCARD_COLLISION_TYPE, ty), handler);
    }
/* TODO
    pub fn set_default_collision_handler<H>(&mut self, mut handler: Box<H>)
        where H: CollisionHandler
    {
        unsafe { 
            load_collision_handler::<H>(
                &mut handler, ffi::cpSpaceAddDefaultHandler(self.as_mut_ptr())
            );
        }
        self.handlers.insert((WILDCARD_COLLISION_TYPE, ty), handler);
    }
*/
    /// Switch the space to use a spatial hash as it's spatial index.
    pub fn use_spatial_hash(&mut self, dim: Scalar, count: i32){
        unsafe { ffi::cpSpaceUseSpatialHash(self.as_mut_ptr(), dim, count); }
    }

    /// Step the space forward in time by @c dt.
    pub fn step(&mut self, dt: Scalar){
        unsafe { ffi::cpSpaceStep(self.as_mut_ptr(), dt); }
    }
}

impl Drop for Space {
    fn drop(&mut self){
        unsafe {
            //TODO do we need to do this?
            let bodies: Vec<BodyHandle> = self.bodies().map(|b| b.handle()).collect();
            for handle in bodies {
                self.remove_body(handle);
            }

            ffi::cpSpaceDestroy(self.as_mut_ptr());
        }
    }
}

macro_rules! iterators {
    ($item:ident, $iter:ident, $iter_mut:ident, $handle:ident) => (

        pub struct $iter<'a> {
            iter: hash_set::Iter<'a, $handle>
        }

        impl<'a> Iterator for $iter<'a> {
            type Item = &'a $item;
            fn next(&mut self) -> Option<Self::Item> {
                self.iter.next().map(|handle|
                    unsafe { $item::from_ptr(handle.as_ptr()) }
                )
            }
        }

        pub struct $iter_mut<'a> {
            iter: hash_set::Iter<'a, $handle>
        }

        impl<'a> Iterator for $iter_mut<'a> {
            type Item = &'a mut $item;
            fn next(&mut self) -> Option<Self::Item> {
                self.iter.next().map(|handle|
                    unsafe { $item::from_mut_ptr(handle.as_mut_ptr()) }
                )
            }
        }                                  
                                            
    );
}

iterators!(Body, Bodies, BodiesMut, BodyHandle);
iterators!(Constraint, Constraints, ConstraintsMut, ConstraintHandle);
iterators!(Shape, Shapes, ShapesMut, ShapeHandle);

