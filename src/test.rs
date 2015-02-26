use na;

#[test]
fn f32_f64_linkage_error(){
    let mut space = super::Space::new();
    unsafe {
        assert_eq!(space.mut_ground_body().position().x, (*(*space.ptr()).staticBody).p.x);
    }
}

#[test]
fn iter_shapes(){
    let mut space = super::Space::new();

    let body = box super::StaticBody::new();
    let shape = box super::CircleShape::new(20.0);

    let body = space.add_body(super::StaticBody::downcast(body));
    space.attach_shape(body, super::CircleShape::downcast(shape));

    assert_eq!(space.body(body).shapes().count(), 1);
}
