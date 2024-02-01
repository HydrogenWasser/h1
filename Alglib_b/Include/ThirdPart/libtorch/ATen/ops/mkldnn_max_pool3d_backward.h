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



#include <ATen/ops/mkldnn_max_pool3d_backward_ops.h>

namespace at {


// aten::mkldnn_max_pool3d_backward(Tensor grad_output, Tensor output, Tensor input, int[3] kernel_size, int[3] stride=[], int[3] padding=0, int[3] dilation=1, bool ceil_mode=False) -> Tensor
TORCH_API inline at::Tensor mkldnn_max_pool3d_backward(const at::Tensor & grad_output, const at::Tensor & output, const at::Tensor & input, at::IntArrayRef kernel_size, at::IntArrayRef stride={}, at::IntArrayRef padding=0, at::IntArrayRef dilation=1, bool ceil_mode=false) {
    return at::_ops::mkldnn_max_pool3d_backward::call(grad_output, output, input, kernel_size, stride, padding, dilation, ceil_mode);
}

}
