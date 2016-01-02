use super::{NO_GROUP, ALL_CATEGORIES};
use super::{Scalar, Pnt2, Vec2, Iso2, Bitmask, CollisionType, Group};
use bb::BB;
//use arbiter::ContactPointSet;
use body::BodyHandle;
use ffi;

use na;

use libc::c_int;
use std::ops::{Deref, DerefMut};
use std::{mem, ptr};

// ++++++++++++++++++++ utils ++++++++++++++++++++

/// Nearest point query info struct.
///
/// NOTE: Doesn't contain `const cpShape *shape;`
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
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
#[derive(Debug, Clone, Copy, PartialEq)]
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
    pub fn new(group: Group, categories: Bitmask, mask: Bitmask) -> ShapeFilter {
        ShapeFilter{ group: group, categories: categories, mask: mask }
    }
    pub fn all() -> ShapeFilter { SHAPE_FILTER_ALL }
}

// ++++++++++++++++++++ Shape-types ++++++++++++++++++++

cp_object!{ (Shape, ShapeHandle, ShapeDowncast, ShapeDowncastRef, ShapeDowncastMut): ffi::cpShape {
    drop: |self_: &mut Shape| {
        unsafe { ffi::cpShapeDestroy(self_.as_mut_ptr()); }
    },
    downcast: |self_: Box<Shape>| -> ShapeDowncast { 
        unsafe {
            match (*self_.__data.klass)._type {
                ffi::CP_CIRCLE_SHAPE => ShapeDowncast::Circle(mem::transmute(self_)),
                ffi::CP_SEGMENT_SHAPE => ShapeDowncast::Segment(mem::transmute(self_)),
                ffi::CP_POLY_SHAPE => ShapeDowncast::Poly(mem::transmute(self_)),
                _ => unreachable!()
            }
        }
    },
    variants: {
        (CircleShape, Circle): ffi::cpCircleShape,
        (SegmentShape, Segment): ffi::cpSegmentShape,
        (PolyShape, Poly): ffi::cpPolyShape,
    }
} }

impl Shape {
    pub unsafe fn set_body(&mut self, body: Option<BodyHandle>){
        ffi::cpShapeSetBody(self.as_mut_ptr(), match body {
            Some(body) => body.as_mut_ptr(),
            None => ptr::null_mut()
        });
    }

    pub fn is_attached(&self) -> bool {
        unsafe { !ffi::cpShapeGetBody(self.as_ptr()).is_null() }
    }

    pub fn body(&self) -> Option<BodyHandle> {
        if self.is_attached() {
            Some(unsafe { BodyHandle::new(ffi::cpShapeGetBody(self.as_ptr()) as *mut _) })
        } else {
            None
        }
    }

    /// Update, cache and return the bounding box of a shape based on the body it's attached to.
    pub fn cache_bb(&mut self) -> BB {
        unsafe { ffi::cpShapeCacheBB(self.as_mut_ptr()) } // TODO nullptr?
    }

    /// Update, cache and return the bounding box of a shape with an explicit transformation.
    pub fn update(&mut self, transform: Iso2) -> BB {
        unsafe { ffi::cpShapeUpdate(self.as_mut_ptr(), transform) } // TODO: nullptr?
    }

    /// Perform a nearest point query. It finds the closest point on the surface of shape to a 
    /// specific point. The value returned is the distance between the points. A negative 
    /// distance means the point is inside the shape.
    pub fn point_query(&self, p: Pnt2) -> PointQueryInfo {
        unsafe {
            let mut val: ffi::cpPointQueryInfo = mem::zeroed();
            ffi::cpShapePointQuery(self.as_ptr(), p.to_vec(), &mut val);
            PointQueryInfo::new(val.point.to_pnt(), val.distance, val.gradient)
        }
    }

    /// Perform a segment query against a shape. @c info must be a pointer to a valid 
    /// cpSegmentQueryInfo structure.
    pub fn segment_query(&self, a: Pnt2, b: Pnt2, radius: Scalar) -> SegmentQueryInfo {
        unsafe {
           let mut val: ffi::cpSegmentQueryInfo = mem::zeroed();
            ffi::cpShapeSegmentQuery(self.as_ptr(), a.to_vec(), b.to_vec(), radius, &mut val);
            SegmentQueryInfo::new(val.point.to_pnt(), val.normal, val.alpha)
        }
    }

    /*
    /// Return contact information about two shapes.
    pub fn collide<S: ShapeDowncast>(&self, other: &S) -> Option<ContactPointSet> {
        unsafe { 
            ffi::cpShapesCollide(self.as_ptr(), other.deref().ptr())
        }
    }
    */

    /// Get the mass of the shape if you are having Chipmunk calculate mass properties for you.
    pub fn mass(&self) -> Scalar {
        unsafe { ffi::cpShapeGetMass(self.as_ptr() as *mut _) } // TODO fix const
    }
    /// Set the mass of this shape to have Chipmunk calculate mass properties for you.
    pub fn set_mass(&mut self, v: Scalar){
        unsafe { ffi::cpShapeSetMass(self.as_mut_ptr(), v); }
    }
    
    /// Get the density of the shape if you are having Chipmunk calculate mass properties for you.
    pub fn density(&self) -> Scalar {
        unsafe { ffi::cpShapeGetDensity(self.as_ptr() as *mut _) } // TODO fix const
    }
    /// Set the density  of this shape to have Chipmunk calculate mass properties for you.
    pub fn set_density(&mut self, v: Scalar) {
        unsafe { ffi::cpShapeSetDensity(self.as_mut_ptr(), v); }
    }

    /// Get the calculated moment of inertia for this shape.
    pub fn moment(&self) -> Scalar {
        unsafe { ffi::cpShapeGetMoment(self.as_ptr() as *mut _) } // TODO fix const
    }
    
    /// Get the calculated area of this shape.
    pub fn area(&self) -> Scalar {
        unsafe { ffi::cpShapeGetArea(self.as_ptr() as *mut _) } // TODO fix const
    }
    
    /// Get the centroid of this shape.
    pub fn center_of_gravity(&self) -> Pnt2 {
        unsafe { ffi::cpShapeGetCenterOfGravity(self.as_ptr() as *mut _).to_pnt() } // TODO fix const
    }
    
    /// Get the bounding box that contains the shape given it's current position and angle.
    pub fn bb(&self) -> BB {
        unsafe { ffi::cpShapeGetBB(self.as_ptr()) }
    }
    
    /// Get if the shape is set to be a sensor or not.
    pub fn sensor(&self) -> bool {
        unsafe { ffi::cpShapeGetSensor(self.as_ptr()) == 1 }
    }
    /// Set if the shape is a sensor or not.
    pub fn set_sensor(&mut self, sensor: bool){
        unsafe { ffi::cpShapeSetSensor(self.as_mut_ptr(), if sensor { 1 } else { 0 }); }
    }
    
    /// Get the elasticity of this shape.
    pub fn elasticity(&self) -> Scalar {
        unsafe { ffi::cpShapeGetElasticity(self.as_ptr()) }
    }
    /// Set the elasticity of this shape.
    pub fn set_elasticity(&mut self, v: Scalar){
        unsafe { ffi::cpShapeSetElasticity(self.as_mut_ptr(), v); }
    }
    
    /// Get the friction of this shape.
    pub fn friction(&self) -> Scalar {
        unsafe { ffi::cpShapeGetFriction(self.as_ptr()) }
    }
    /// Set the friction of this shape.
    pub fn set_friction(&mut self, v: Scalar){
        unsafe { ffi::cpShapeSetFriction(self.as_mut_ptr(), v); }
    }
    
    /// Get the surface velocity of this shape.
    pub fn surface_velocity(&self) -> Vec2 {
        unsafe { ffi::cpShapeGetSurfaceVelocity(self.as_ptr()) }
    }
    /// Set the surface velocity of this shape.
    pub fn set_surface_velocity(&mut self, v: Vec2){
        unsafe { ffi::cpShapeSetSurfaceVelocity(self.as_mut_ptr(), v); }
    }

    /// Set the collision type of this shape.
    pub fn collision_type(&self) -> CollisionType {
        unsafe { ffi::cpShapeGetCollisionType(self.as_ptr()) }
    }
    /// Get the collision type of this shape.
    pub fn set_collision_type(&mut self, v: CollisionType){
        unsafe { ffi::cpShapeSetCollisionType(self.as_mut_ptr(), v); }
    }

    /// Get the collision filtering parameters of this shape.
    pub fn filter(&self) -> ShapeFilter {
        unsafe { ffi::cpShapeGetFilter(self.as_ptr()) }
    }
    /// Set the collision filtering parameters of this shape.
    pub fn set_filter(&mut self, filter: ShapeFilter){
        unsafe { ffi::cpShapeSetFilter(self.as_mut_ptr(), filter); }
    }
}

impl CircleShape {
    pub fn new(radius: Scalar) -> Self {
        CircleShape::with_offset(radius, na::zero())
    }

    pub fn with_offset(radius: Scalar, offset: Vec2) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpCircleShapeInit(&mut data, ptr::null_mut(), radius, offset);
            CircleShape{ __data: data }
        }
    }

    /// Get the offset of a circle shape.
    pub fn offset(&self) -> Vec2 {
        unsafe { ffi::cpCircleShapeGetOffset(self.as_ptr()) }
    }

    /// Get the radius of a circle shape.
    pub fn radius(&self) -> Scalar {
        unsafe { ffi::cpCircleShapeGetRadius(self.as_ptr()) }
    }
}

impl SegmentShape {
    pub fn new(a: Pnt2, b: Pnt2, thickness: Scalar) -> Self {
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpSegmentShapeInit(&mut data, ptr::null_mut(), a.to_vec(), b.to_vec(), thickness);
            SegmentShape{ __data: data }
        }
    }

    /// Let Chipmunk know about the geometry of adjacent segments to avoid colliding with endcaps.
    pub fn set_neighbors(&mut self, prev: Pnt2, next: Pnt2){
        unsafe { ffi::cpSegmentShapeSetNeighbors(self.as_mut_ptr(), prev.to_vec(), next.to_vec()); }
    }

    pub fn point_a(&self) -> Pnt2 {
        unsafe { ffi::cpSegmentShapeGetA(self.as_ptr()).to_pnt() }
    }

    pub fn point_b(&self) -> Pnt2 {
        unsafe { ffi::cpSegmentShapeGetB(self.as_ptr()).to_pnt() }
    }

    pub fn normal(&self) -> Vec2 {
        unsafe { ffi::cpSegmentShapeGetNormal(self.as_ptr()) }
    }

    pub fn thickness(&self) -> Scalar {
        unsafe { ffi::cpSegmentShapeGetRadius(self.as_ptr()) }
    }
}

impl PolyShape {
    // TODO cpPolyValidate

    /// Creates a polygon shape with rounded corners.
    /// A convex hull will be created from the vertexes.
    pub fn new(verts: &[Pnt2], transform: Option<Iso2>, border_radius: Option<Scalar>)-> Self {
        let transform = transform.unwrap_or(na::one());
        let border_radius = border_radius.unwrap_or(0.0);
        unsafe {
            let verts = mem::transmute::<&[Pnt2], &[Vec2]>(verts);
            let mut data = mem::zeroed();
            ffi::cpPolyShapeInit(&mut data, ptr::null_mut(), verts.len() as c_int, verts.as_ptr(), transform, border_radius);
            PolyShape{ __data: data }
        }
    }

    /// Creates a box shaped polygon shape.
    pub fn centered_box(width: Scalar, height: Scalar, border_radius: Option<Scalar>) -> Self {
        let border_radius = border_radius.unwrap_or(0.0);
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpBoxShapeInit(&mut data, ptr::null_mut(), width, height, border_radius);
            PolyShape{ __data: data }
        }
    }

    /// Creates an offset box shaped polygon shape.
    pub fn from_bb(bb: BB, border_radius: Option<Scalar>) -> Self {
        let border_radius = border_radius.unwrap_or(0.0);
        unsafe {
            let mut data = mem::zeroed();
            ffi::cpBoxShapeInit2(&mut data, ptr::null_mut(), bb, border_radius);
            PolyShape{ __data: data }
        }
    }

    /// Get the number of verts in a polygon shape.
    ///
    /// TODO: can we get a slice here?
    pub fn count(&self) -> usize {
        unsafe { ffi::cpPolyShapeGetCount(self.as_ptr()) as usize }
    }

    /// Get the @c ith vertex of a polygon shape.
    ///
    /// Panics if `idx` is too large.
    ///
    /// TODO: can we get a slice here?
    pub fn vertex(&self, idx: usize) -> Pnt2 {
        debug_assert!(idx < self.count());
        unsafe { ffi::cpPolyShapeGetVert(self.as_ptr(), idx as c_int).to_pnt() }
    }

    /// Get the border radius of a polygon shape.
    pub fn border_radius(&self) -> Scalar {
        unsafe { ffi::cpPolyShapeGetRadius(self.as_ptr()) }
    }
}

