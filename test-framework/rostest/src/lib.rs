use proc_macro::*;

#[proc_macro_attribute]
pub fn rostest(attr: TokenStream, item: TokenStream) -> TokenStream {
    item
}
