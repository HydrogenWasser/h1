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



#include <ATen/ops/gather_ops.h>

namespace at {


// aten::gather.out(Tensor self, int dim, Tensor index, *, bool sparse_grad=False, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & gather_out(at::Tensor & out, const at::Tensor & self, int64_t dim, const at::Tensor & index, bool sparse_grad=false) {
    return at::_ops::gather_out::call(self, dim, index, sparse_grad, out);
}

// aten::gather.out(Tensor self, int dim, Tensor index, *, bool sparse_grad=False, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & gather_outf(const at::Tensor & self, int64_t dim, const at::Tensor & index, bool sparse_grad, at::Tensor & out) {
    return at::_ops::gather_out::call(self, dim, index, sparse_grad, out);
}

// aten::gather(Tensor self, int dim, Tensor index, *, bool sparse_grad=False) -> Tensor
TORCH_API inline at::Tensor gather(const at::Tensor & self, int64_t dim, const at::Tensor & index, bool sparse_grad=false) {
    return at::_ops::gather::call(self, dim, index, sparse_grad);
}

// aten::gather.dimname_out(Tensor self, Dimname dim, Tensor index, *, bool sparse_grad=False, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & gather_out(at::Tensor & out, const at::Tensor & self, at::Dimname dim, const at::Tensor & index, bool sparse_grad=false) {
    return at::_ops::gather_dimname_out::call(self, dim, index, sparse_grad, out);
}

// aten::gather.dimname_out(Tensor self, Dimname dim, Tensor index, *, bool sparse_grad=False, Tensor(a!) out) -> Tensor(a!)
TORCH_API inline at::Tensor & gather_outf(const at::Tensor & self, at::Dimname dim, const at::Tensor & index, bool sparse_grad, at::Tensor & out) {
    return at::_ops::gather_dimname_out::call(self, dim, index, sparse_grad, out);
}

// aten::gather.dimname(Tensor self, Dimname dim, Tensor index, *, bool sparse_grad=False) -> Tensor
TORCH_API inline at::Tensor gather(const at::Tensor & self, at::Dimname dim, const at::Tensor & index, bool sparse_grad=false) {
    return at::_ops::gather_dimname::call(self, dim, index, sparse_grad);
}

}
