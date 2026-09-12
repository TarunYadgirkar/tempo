// test_shader_compile.mm — compiles the embedded shaders.metal source on the
// real Metal device and builds every pipeline the renderer uses, with the same
// pixel formats. Catches MSL errors and pipeline-descriptor mismatches
// headlessly (no window server needed), which the runtime
// newLibraryWithSource path would otherwise only surface on a live launch.

#import <Metal/Metal.h>

#include <cstdio>

#include "shaders_metal.h"

static int g_failures = 0;

#define CHECK_MSG(cond, msg)                                              \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::fprintf(stderr, "FAIL %s:%d: %s (%s)\n", __FILE__,       \
                         __LINE__, #cond, (msg));                         \
            g_failures++;                                                 \
        }                                                                 \
    } while (0)

int main() {
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (!device) {
            std::printf(
                "test_shader_compile: no Metal device — skipping\n");
            return 0;
        }

        NSError *err = nil;
        id<MTLLibrary> lib = [device
            newLibraryWithSource:[NSString stringWithUTF8String:
                                               MAC_SHELL_SHADER_SOURCE]
                         options:nil
                           error:&err];
        CHECK_MSG(lib != nil,
                  err ? err.localizedDescription.UTF8String : "?");
        if (!lib)
            return 1;

        auto make_pipeline = [&](NSString *vfn, NSString *ffn, bool blend) {
            MTLRenderPipelineDescriptor *d =
                [[MTLRenderPipelineDescriptor alloc] init];
            d.vertexFunction = [lib newFunctionWithName:vfn];
            d.fragmentFunction = [lib newFunctionWithName:ffn];
            CHECK_MSG(d.vertexFunction != nil, vfn.UTF8String);
            CHECK_MSG(d.fragmentFunction != nil, ffn.UTF8String);
            d.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
            d.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
            if (blend) {
                d.colorAttachments[0].blendingEnabled = YES;
                d.colorAttachments[0].sourceRGBBlendFactor =
                    MTLBlendFactorSourceAlpha;
                d.colorAttachments[0].destinationRGBBlendFactor =
                    MTLBlendFactorOneMinusSourceAlpha;
            }
            NSError *perr = nil;
            id<MTLRenderPipelineState> p =
                [device newRenderPipelineStateWithDescriptor:d error:&perr];
            CHECK_MSG(p != nil,
                      perr ? perr.localizedDescription.UTF8String
                           : vfn.UTF8String);
        };

        // Same pipeline set as ShellRenderer initWithView.
        make_pipeline(@"passthrough_vertex", @"passthrough_fragment", false);
        make_pipeline(@"panel_vertex_main", @"panel_fragment_main", true);
        make_pipeline(@"panel_vertex_main", @"blit_fragment", true);
        make_pipeline(@"panel_vertex_main", @"shadow_fragment", true);
        make_pipeline(@"panel_vertex_main", @"glass_fragment", true);
        make_pipeline(@"key_vertex_main", @"key_fragment_main", true);
        make_pipeline(@"sphere_vertex_main", @"sphere_fragment_main", true);
        make_pipeline(@"sphere_vertex_main", @"sphere_glow_fragment_main", true);
        make_pipeline(@"capsule_vertex_main", @"capsule_fragment_main", true);
        make_pipeline(@"solid_vertex_main", @"solid_fragment_main", true);
    }

    if (g_failures) {
        std::fprintf(stderr, "test_shader_compile: %d FAILURES\n",
                     g_failures);
        return 1;
    }
    std::printf("test_shader_compile: all pipelines built\n");
    return 0;
}
