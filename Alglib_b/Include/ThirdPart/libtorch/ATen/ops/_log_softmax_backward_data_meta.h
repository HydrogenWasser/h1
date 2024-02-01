#pragma once

// @generated by tools/codegen/gen.py from NativeMetaFunction.h

#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <c10/util/Optional.h>
#include <c10/core/QScheme.h>
#include <ATen/core/Reduction.h>
#include <ATen/TensorIterator.h>
#include <ATen/TensorMeta.h>
#include <tuple>
#include <vector>

namespace at {
namespace meta {

struct TORCH_API structured__log_softmax_backward_data : public at::impl::MetaBase {
    
    
    void meta(const at::Tensor & grad_output, const at::Tensor & output, int64_t dim, at::ScalarType input_dtype);
};

} // namespace native
} // namespace at
