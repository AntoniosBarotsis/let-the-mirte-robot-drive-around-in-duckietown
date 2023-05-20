use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

/// Currently marks the method as a test that will be ran serially.
#[proc_macro_attribute]
pub fn ros_test(_attr: TokenStream, item: TokenStream) -> TokenStream {
  // Parse the input function
  let input = parse_macro_input!(item as ItemFn);

  // Generate some new code to replace the original function
  let new_code = quote! {
      #[test]
      #[serial_test::serial]
      #input
  };

  // Return the new code as a TokenStream
  TokenStream::from(new_code)
}
