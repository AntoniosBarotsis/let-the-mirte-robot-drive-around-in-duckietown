use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

/// Currently marks the method as a test that will be ran serially.
#[proc_macro_attribute]
pub fn ros_test(_attr: TokenStream, item: TokenStream) -> TokenStream {
  // Parse the input function
  let mut input = parse_macro_input!(item as ItemFn);

  let init_code = quote!(let _lifetime_variable = test_framework::init(););
  let init_element = syn::parse(TokenStream::from(init_code))
    .expect("Could not create init() syntax element to insert into AST");
  input.block.stmts.insert(0, init_element);

  // Generate some new code to replace the original function
  let new_code = quote! {
      #[test]
      #[serial_test::serial]
      #input
  };

  // Return the new code as a TokenStream
  TokenStream::from(new_code)
}
