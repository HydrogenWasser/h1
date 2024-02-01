#pragma once

// @generated by tools/codegen/gen.py from Function.h

#include <ATen/Context.h>
#include <ATen/DeviceGuard.h>
#include <ATen/TensorUtils.h>
#include <ATen/TracerMode.h>
#include <ATen/core/Generator.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <c10/util/Optional.h>



#include <ATen/ops/dstack_ops.h>

namespace at {


// aten::dstack(Tensor[] tensors) -> Tensor
TORCH_API inline at::Tensor dstack(at::TensorList tensors) {
    return at::_ops::dstack::call(tensors);
}

// aten::dstack.out(Tensor[] tensors, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & dstack_out(at::Tensor & out, at::TensorList tensors) {
    return at::_ops::dstack_out::call(tensors, out);
}

// aten::dstack.out(Tensor[] tensors, *, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & dstack_outf(at::TensorList tensors, at::Tensor & out) {
    return at::_ops::dstack_out::call(tensors, out);
}

}
