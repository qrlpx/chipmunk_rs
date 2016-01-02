
macro_rules! cp_object {
    (($name:ident, $handle:ident, $dc_enum:ident, $dc_enum_ref:ident, $dc_enum_mut:ident): $cp_ty:ty {
        drop: $drop_fn:expr, 
        downcast: $dc_fn:expr,
        variants: {
            $(($variant_name:ident, $variant_enum_name:ident): $variant_cp_ty:ty,)+
        }
     }) => {

        #[derive(Debug, Hash, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
        pub struct $handle {
            __ptr: ::core::nonzero::NonZero<*const $cp_ty>
        }

        impl $handle {
            #[doc(hidden)]
            pub fn new(ptr: *const $cp_ty) -> Self { 
                debug_assert!(!ptr.is_null());
                $handle{ __ptr: unsafe { ::core::nonzero::NonZero::new(ptr) } }
            }

            #[doc(hidden)]
            pub fn as_ptr(&self) -> *const $cp_ty { *self.__ptr }

            #[doc(hidden)]
            pub fn as_mut_ptr(&self) -> *mut $cp_ty { *self.__ptr as *mut _ }
        }

        pub enum $dc_enum {
            $($variant_enum_name(Box<$variant_name>)),+
        }

        pub enum $dc_enum_ref<'a> {
            $($variant_enum_name(&'a $variant_name)),+
        }

        pub enum $dc_enum_mut<'a> {
            $($variant_enum_name(&'a mut $variant_name)),+
        }
        
        pub struct $name {
            __data: $cp_ty
        }

        impl $name {
            #[doc(hidden)]
            pub unsafe fn from_ptr(ptr: *const $cp_ty) -> &'static Self {
                ::std::mem::transmute(ptr)
            }

            #[doc(hidden)]
            pub unsafe fn from_mut_ptr(ptr: *mut $cp_ty) -> &'static mut Self {
                ::std::mem::transmute(ptr)
            }

            #[doc(hidden)]
            pub unsafe fn from_owned_ptr(ptr: *mut $cp_ty) -> Box<Self> {
                ::std::mem::transmute(ptr)
            }

            #[doc(hidden)]
            pub fn as_ptr(&self) -> *const $cp_ty { &self.__data }

            #[doc(hidden)]
            pub fn as_mut_ptr(&mut self) -> *mut $cp_ty { &mut self.__data }

            /// Note that all handles are just wrapped pointers, so take care to
            /// call this method only on boxed objects. 
            pub fn handle(&self) -> $handle { $handle::new(self.as_ptr()) }

            pub fn downcast(self: Box<Self>) -> $dc_enum {
                ($dc_fn)(self)
            }

            pub fn downcast_ref(&self) -> $dc_enum_ref {
                use std::mem;
                unsafe {
                    let owned = mem::transmute::<&Self, Box<Self>>(self);
                    let owned_dc = ($dc_fn)(owned);
                    mem::transmute::<$dc_enum, $dc_enum_ref>(owned_dc)
                }
            }

            pub fn downcast_mut(&mut self) -> $dc_enum_mut {
                use std::mem;
                unsafe {
                    let owned = mem::transmute::<&mut Self, Box<Self>>(self);
                    let owned_dc = ($dc_fn)(owned);
                    mem::transmute::<$dc_enum, $dc_enum_mut>(owned_dc)
                }
            }
        }

        impl Drop for $name {
            fn drop(&mut self){
                ($drop_fn)(self)
            }
        }

        $(
            pub struct $variant_name {
                __data: $variant_cp_ty
            }

            impl ::std::ops::Deref for $variant_name {
                type Target = $name;
                fn deref(&self) -> &Self::Target {
                    unsafe { $name::from_ptr(&self.__data as *const $variant_cp_ty as *const $cp_ty) }
                }
            }

            impl ::std::ops::DerefMut for $variant_name {
                fn deref_mut(&mut self) -> &mut Self::Target {
                    unsafe { $name::from_mut_ptr(&mut self.__data as *mut $variant_cp_ty as *mut $cp_ty) }
                }
            }

            impl Into<Box<$name>> for Box<$variant_name> {
                fn into(self) -> Box<$name> {
                    use std::mem;
                    unsafe { mem::transmute(self) }
                }
            }

            impl Drop for $variant_name {
                fn drop(&mut self){
                    ($drop_fn)(self)
                }
            }
        )+

    };
}
