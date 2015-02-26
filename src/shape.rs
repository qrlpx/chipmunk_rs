use super::{NO_GROUP, ALL_CATEGORIES};
use super::{Scalar, Pnt2, Vec2, Iso2, BitMask, CollisionType, Group, ObjectHandle};
use bb::BB;
use arbiter::ContactPointSet;
use body::{BodyHandle, BodyBase};
use ffi;

use na;

use libc::c_int;
use std::ops::{Deref, DerefMut};
use std::{mem, ptr};

/// Nearest point query info struct.
///
/// NOTE: Doesn't contain `const cpShape *shape;`
#[repr(C)]
#[derive(Copy)]
pub struct PointQueryInfo {
	/// The closest point on the shape's surface. (in world space coordinates)
	pub point: Pnt2,
	/// The distance to the point. The distance is negative if the point is inside the shape.
	pub distance: Scalar,
	/// The gradient of the signed distance function.
	/// The same as info.p/info.d, but accurate even for very small values of info.d.
	pub gradient: Vec2
}

impl PointQueryInfo {
    pub fn new(point: Pnt2, distance: Scalar, gradient: Vec2) -> Self {
        PointQueryInfo{ point: point, distance: distance, gradient: gradient }
    }
}

/// Segment query info struct.
///
/// NOTE: Doesn't contain `const cpShape *shape;`
#[repr(C)]
#[derive(Copy)]
pub struct SegmentQueryInfo {
	/// The point of impact.
	pub point: Pnt2,
	/// The normal of the surface hit.
	pub normal: Vec2,
	/// The normalized distance along the query segment in the range [0, 1].
	pub alpha: Scalar
}

impl SegmentQueryInfo {
    pub fn new(point: Pnt2, normal: Vec2, alpha: Scalar) -> Self {
        SegmentQueryInfo{ point: point, normal: normal, alpha: alpha }
    }
}

pub type ShapeFilter = ffi::cpShapeFilter;

pub const SHAPE_FILTER_ALL: ShapeFilter = ShapeFilter{ 
    group: NO_GROUP, 
    categories: ALL_CATEGORIES, 
    mask: ALL_CATEGORIES,
};

impl ShapeFilter {
    pub fn new(group: Group, categories: BitMask, mask: BitMask) -> ShapeFilter {
        ShapeFilter{ group: group, categories: categories, mask: mask }
    }
    pub fn all() -> ShapeFilter { SHAPE_FILTER_ALL }
}

pub enum ShapeUpcast {
    Circle(Box<CircleShape>),
    Segment(Box<SegmentShape>),
    Poly(Box<PolyShape>),
}

pub enum ShapeUpcastRef<'a> {
    Circle(&'a CircleShape),
    Segment(&'a SegmentShape),
    Poly(&'a PolyShape),
}

pub enum ShapeUpcastMut<'a> {
    Circle(&'a mut CircleShape),
    Segment(&'a mut SegmentShape),
    Poly(&'a mut PolyShape),
}

pub type ShapeHandle = ObjectHandle<ffi::cpShape>;

pub struct ShapeBase(ffi::cpShape);

unsafe impl Sync for ShapeBase {}

impl ShapeBase {
    pub unsafe fn from_ptr(ptr: *const ffi::cpShape) -> &'static ShapeBase {
        mem::transmute(ptr)
    }
    pub unsafe fn from_mut_ptr(ptr: *mut ffi::cpShape) -> &'static mut ShapeBase {
        mem::transmute(ptr)
    }
    pub unsafe fn box_from_ptr(ptr: *const ffi::cpShape) -> Box<ShapeBase> {
        mem::transmute(ptr)
    }

    pub fn upcast(_self: Box<Self>) -> ShapeUpcast {
        unsafe {
            match (*_self.0.klass)._type {
                ffi::CP_CIRCLE_SHAPE => ShapeUpcast::Circle(mem::transmute(_self)),
                ffi::CP_SEGMENT_SHAPE => ShapeUpcast::Segment(mem::transmute(_self)),
                ffi::CP_POLY_SHAPE => ShapeUpcast::Poly(mem::transmute(_self)),
                _ => unreachable!()
            }
        }
    }
    pub fn upcast_ref(&self) -> ShapeUpcastRef {
        unsafe { mem::transmute(ShapeBase::upcast(mem::transmute(self))) }
    }
    pub fn upcast_mut(&mut self) -> ShapeUpcastMut {
        unsafe { mem::transmute(ShapeBase::upcast(mem::transmute(self))) }
    }

    pub fn ptr(&self) -> *const ffi::cpShape { &self.0 }
    pub fn mut_ptr(&mut self) -> *mut ffi::cpShape { &mut self.0 }
    pub fn handle(&self) -> ShapeHandle {
        ObjectHandle::wrap(self.ptr() as *mut _)
    }

    pub unsafe fn set_body(&mut self, body: Option<BodyHandle>){
        ffi::cpShapeSetBody(self.mut_ptr(), match body {
            Some(body) => body.unwrap(),
            None => ptr::null_mut()
        });
    }

    pub fn is_attached(&self) -> bool {
        unsafe { ffi::cpShapeGetBody(self.ptr()).is_null() == false }
    }

    pub fn body(&self) -> &BodyBase {
        debug_assert!(self.is_attached());
        unsafe { 
            let body = ffi::cpShapeGetBody(self.ptr()) as *mut ffi::cpBody;
            BodyBase::from_ptr(body)
        }
    }
    pub fn mut_body(&mut self) -> &mut BodyBase {
        unsafe { mem::transmute(self.body()) }
    }

    /// Update, cache and return the bounding box of a shape based on the body it's attached to.
    pub fn cache_bb(&mut self) -> BB {
        unsafe { ffi::cpShapeCacheBB(self.mut_ptr()) } // TODO nullptr?
    }

    /// Update, cache and return the bounding box of a shape with an explicit transformation.
    pub fn update(&mut self, transform: Iso2) -> BB {
        unsafe { ffi::cpShapeUpdate(self.mut_ptr(), transform) } // TODO: nullptr?
    }

    /// Perform a nearest point query. It finds the closest point on the surface of shape to a 
    /// specific point. The value returned is the distance between the points. A negative 
    /// distance means the point is inside the shape.
    pub fn point_query(&self, p: Pnt2) -> PointQueryInfo {
        unsafe {
            let mut val: ffi::cpPointQueryInfo = mem::zeroed();
            ffi::cpShapePointQuery(self.ptr(), p, &mut val);
            PointQueryInfo::new(val.point, val.distance, val.gradient.to_vec())
        }
    }

    /// Perform a segment query against a shape. @c info must be a pointer to a valid 
    /// cpSegmentQueryInfo structure.
    pub fn segment_query(&self, a: Pnt2, b: Pnt2, radius: Scalar) -> SegmentQueryInfo {
        unsafe {
           let mut val: ffi::cpSegmentQueryInfo = mem::zeroed();
            ffi::cpShapeSegmentQuery(self.ptr(), a, b, radius, &mut val);
            SegmentQueryInfo::new(val.point, val.normal.to_vec(), val.alpha)
        }
    }

    /*
    /// Return contact information about two shapes.
    pub fn collide<S: ShapeDowncast>(&self, other: &S) -> Option<ContactPointSet> {
        unsafe { 
            ffi::cpShapesCollide(self.ptr(), other.deref().ptr())
        }
    }
    */

    /// Get the mass of the shape if you are having Chipmunk calculate mass properties for you.
    pub fn mass(&self) -> Scalar {
        unsafe { ffi::cpShapeGetMass(self.ptr() as *mut _) } // TODO fix const
    }
    /// Set the mass of this shape to have Chipmunk calculate mass properties for you.
    pub fn set_mass(&mut self, v: Scalar){
        unsafe { ffi::cpShapeSetMass(self.mut_ptr(), v); }
    }
    
    /// Get the density of the shape if you are having Chipmunk calculate mass properties for you.
    pub fn density(&self) -> Scalar {
        unsafe { ffi::cpShapeGetDensity(self.ptr() as *mut _) } // TODO fix const
    }
    /// Set the density  of this shape to have Chipmunk calculate mass properties for you.
    pub fn set_density(&mut self, v: Scalar) {
        unsafe { ffi::cpShapeSetDensity(self.mut_ptr(), v); }
    }

    /// Get the calculated moment of inertia for this shape.
    pub fn moment(&self) -> Scalar {
        unsafe { ffi::cpShapeGetMoment(self.ptr() as *mut _) } // TODO fix const
    }
    
    /// Get the calculated area of this shape.
    pub fn area(&self) -> Scalar {
        unsafe { ffi::cpShapeGetArea(self.ptr() as *mut _) } // TODO fix const
    }
    
    /// Get the centroid of this shape.
    pub fn center_of_gravity(&self) -> Pnt2 {
        unsafe { ffi::cpShapeGetCenterOfGravity(self.ptr() as *mut _) } // TODO fix const
    }
    
    /// Get the bounding box that contains the shape given it's current position and angle.
    pub fn bb(&self) -> BB {
        unsafe { ffi::cpShapeGetBB(self.ptr()) }
    }
    
    /// Get if the shape is set to be a sensor or not.
    pub fn sensor(&self) -> bool {
        unsafe { ffi::cpShapeGetSensor(self.ptr()) == 1 }
    }
    /// Set if the shape is a sensor or not.
    pub fn set_sensor(&mut self, sensor: bool){
        unsafe { ffi::cpShapeSetSensor(self.mut_ptr(), if sensor { 1 } else { 0 }); }
    }
    
    /// Get the elasticity of this shape.
    pub fn elasticity(&self) -> Scalar {
        unsafe { ffi::cpShapeGetElasticity(self.ptr()) }
    }
    /// Set the elasticity of this shape.
    pub fn set_elasticity(&mut self, v: Scalar){
        unsafe { ffi::cpShapeSetElasticity(self.mut_ptr(), v); }
    }
    
    /// Get the friction of this shape.
    pub fn friction(&self) -> Scalar {
        unsafe { ffi::cpShapeGetFriction(self.ptr()) }
    }
    /// Set the friction of this shape.
    pub fn set_friction(&mut self, v: Scalar){
        unsafe { ffi::cpShapeSetFriction(self.mut_ptr(), v); }
    }
    
    /// Get the surface velocity of this shape.
    pub fn surface_velocity(&self) -> Vec2 {
        unsafe { ffi::cpShapeGetSurfaceVelocity(self.ptr()).to_vec() }
    }
    /// Set the surface velocity of this shape.
    pub fn set_surface_velocity(&mut self, v: Vec2){
        unsafe { ffi::cpShapeSetSurfaceVelocity(self.mut_ptr(), na::orig::<Pnt2>() + v); }
    }

    /// Set the collision type of this shape.
    pub fn collision_type(&self) -> CollisionType {
        unsafe { ffi::cpShapeGetCollisionType(self.ptr()) }
    }
    /// Get the collision type of this shape.
    pub fn set_collision_type(&mut self, v: CollisionType){
        unsafe { ffi::cpShapeSetCollisionType(self.mut_ptr(), v); }
    }

    /// Get the collision filtering parameters of this shape.
    pub fn filter(&self) -> ShapeFilter {
        unsafe { ffi::cpShapeGetFilter(self.ptr()) }
    }
    /// Set the collision filtering parameters of this shape.
    pub fn set_filter(&mut self, filter: ShapeFilter){
        unsafe { ffi::cpShapeSetFilter(self.mut_ptr(), filter); }
    }

}

impl Drop for ShapeBase {
    fn drop(&mut self){
        unsafe { ffi::cpShapeDestroy(self.mut_ptr()); }
    }
}

macro_rules! impl_shape {
    ($ty:ty) => (
        impl $ty {
            pub fn downcast(_self: Box<Self>) -> Box<ShapeBase> {
                unsafe { mem::transmute(_self) }
            }
        }
        unsafe impl Sync for $ty {}
        impl Deref for $ty {
            type Target = ShapeBase;
            fn deref(&self) -> &ShapeBase { 
                unsafe { mem::transmute(self) }
            }
        }
        impl DerefMut for $ty {
            fn deref_mut(&mut self) -> &mut ShapeBase { 
                unsafe { mem::transmute(self) }
            }
        }
        impl Drop for $ty {
            fn drop(&mut self){
                unsafe { ffi::cpShapeDestroy(self.mut_ptr()); }
            }
        }
    );
}

pub struct CircleShape(ffi::cpCircleShape);
impl_shape!(CircleShape);

impl CircleShape {
    pub fn new(radius: Scalar) -> Self {
        CircleShape::with_offset(radius, na::orig())
    }

    pub fn with_offset(radius: Scalar, offset: Pnt2) -> Self {
        CircleShape(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpCircleShapeInit(&mut data, ptr::null_mut(), radius, offset);
            data
        })
    }

    /// Get the offset of a circle shape.
    pub fn offset(&self) -> Pnt2 {
        unsafe { ffi::cpCircleShapeGetOffset(self.ptr()) }
    }

    /// Get the radius of a circle shape.
    pub fn radius(&self) -> Scalar {
        unsafe { ffi::cpCircleShapeGetRadius(self.ptr()) }
    }
}

pub struct SegmentShape(ffi::cpSegmentShape);
impl_shape!(SegmentShape);

impl SegmentShape {
    // TODO cpPolyValidate

    pub fn new(a: Pnt2, b: Pnt2, thickness: Scalar) -> Self {
        SegmentShape(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpSegmentShapeInit(&mut data, ptr::null_mut(), a, b, thickness);
            data
        })
    }

    /// Let Chipmunk know about the geometry of adjacent segments to avoid colliding with endcaps.
    pub fn set_neighbors(&mut self, prev: Pnt2, next: Pnt2){
        unsafe { ffi::cpSegmentShapeSetNeighbors(self.mut_ptr(), prev, next); }
    }

    pub fn point_a(&self) -> Pnt2 {
        unsafe { ffi::cpSegmentShapeGetA(self.ptr()) }
    }

    pub fn point_b(&self) -> Pnt2 {
        unsafe { ffi::cpSegmentShapeGetB(self.ptr()) }
    }

    pub fn normal(&self) -> Pnt2 {
        unsafe { ffi::cpSegmentShapeGetNormal(self.ptr()) }
    }

    pub fn thickness(&self) -> Scalar {
        unsafe { ffi::cpSegmentShapeGetRadius(self.ptr()) }
    }
}


pub struct PolyShape(ffi::cpPolyShape);
impl_shape!(PolyShape);

impl PolyShape {
    /// Creates a polygon shape with rounded corners.
    /// A convex hull will be created from the vertexes.
    pub fn new(verts: &[Pnt2], transform: Option<Iso2>, border_radius: Option<Scalar>)
        -> Self
    {
        let transform = transform.unwrap_or(na::one());
        let border_radius = border_radius.unwrap_or(0.0);
        PolyShape(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpPolyShapeInit(
                &mut data, ptr::null_mut(), 
                verts.len() as c_int, verts.as_ptr(),
                transform, border_radius
            );
            data
        })
    }

    /// Creates a box shaped polygon shape.
    pub fn centered_box(width: Scalar, height: Scalar, border_radius: Option<Scalar>)
        -> Self
    {
        let border_radius = border_radius.unwrap_or(0.0);
        PolyShape(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpBoxShapeInit(&mut data, ptr::null_mut(), width, height, border_radius);
            data
        })
    }

    /// Creates an offset box shaped polygon shape.
    pub fn from_bb(bb: BB, border_radius: Option<Scalar>) -> Self {
        let border_radius = border_radius.unwrap_or(0.0);
        PolyShape(unsafe {
            let mut data = mem::uninitialized();
            ffi::cpBoxShapeInit2(&mut data, ptr::null_mut(), bb, border_radius);
            data
        })
    }

    /// Get the number of verts in a polygon shape.
    ///
    /// TODO: replace with (RandomAccess)Iterator?
    pub fn count(&self) -> usize {
        unsafe { ffi::cpPolyShapeGetCount(self.ptr()) as usize }
    }

    /// Get the @c ith vertex of a polygon shape.
    ///
    /// Panics if `idx` is too large.
    ///
    /// TODO: replace with (RandomAccess)Iterator?
    pub fn vertex(&self, idx: usize) -> Pnt2 {
        debug_assert!(idx < self.count());
        unsafe { ffi::cpPolyShapeGetVert(self.ptr(), idx as c_int) }
    }

    /// Get the border radius of a polygon shape.
    pub fn border_radius(&self) -> Scalar {
        unsafe { ffi::cpPolyShapeGetRadius(self.ptr()) }
    }
}
