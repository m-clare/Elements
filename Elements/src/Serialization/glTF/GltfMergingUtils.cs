using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using glTFLoader;
using glTFLoader.Schema;

[assembly: InternalsVisibleTo("Elements.Tests")]
namespace Elements.Serialization.glTF
{

    internal static class GltfMergingUtils
    {
        public static List<int> AddAllMeshesFromFromGlb(Stream glbStream,
                                        List<Buffer> buffers,
                                        List<byte[]> bufferByteArrays,
                                        List<BufferView> bufferViews,
                                        List<Accessor> accessors,
                                        List<glTFLoader.Schema.Mesh> meshes,
                                        List<glTFLoader.Schema.Material> materials,
                                        List<Texture> textures,
                                        List<Image> images,
                                        List<Sampler> samplers,
                                        bool shouldAddMaterials
                                        )
        {
            var loadingStream = new MemoryStream();
            glbStream.Position = 0;
            glbStream.CopyTo(loadingStream);
            loadingStream.Position = 0;
            var loaded = Interface.LoadModel(loadingStream);
            var newByteArrays = loaded.GetAllBufferByteArrays(glbStream);
            bufferByteArrays.AddRange(newByteArrays);

            var bufferIncrement = buffers.Count;
            foreach (var originBuffer in loaded.Buffers)
            {
                buffers.Add(originBuffer);
            }

            var buffViewIncrement = bufferViews.Count;
            foreach (var originBuffView in loaded.BufferViews)
            {
                originBuffView.Buffer = originBuffView.Buffer + bufferIncrement;
                bufferViews.Add(originBuffView);
            }

            var accessorIncrement = accessors.Count;
            foreach (var originAccessor in loaded.Accessors)
            {
                originAccessor.BufferView = originAccessor.BufferView + buffViewIncrement;
                accessors.Add(originAccessor);
            }

            var imageIncrement = images.Count;
            if (loaded.Images != null)
            {
                foreach (var originImage in loaded.Images)
                {
                    if (originImage.BufferView.HasValue)
                    {
                        originImage.BufferView = originImage.BufferView + buffViewIncrement;
                    }
                    images.Add(originImage);
                }
            }

            var samplerIncrement = samplers.Count;
            if (loaded.Samplers != null)
            {
                foreach (var originSampler in loaded.Samplers)
                {
                    samplers.Add(originSampler);
                }
            }

            var textureIncrement = textures.Count;
            if (loaded.Textures != null)
            {
                foreach (var originTexture in loaded.Textures)
                {
                    originTexture.Source = originTexture.Source + imageIncrement;
                    if (originTexture.Sampler.HasValue)
                    {
                        originTexture.Sampler = originTexture.Sampler + samplerIncrement;

                    }
                    textures.Add(originTexture);
                }
            }


            var materialIncrement = materials.Count;

            if (shouldAddMaterials)
            {
                AddMaterials(materials, loaded, textureIncrement);
            }

            var meshIndices = new List<int>();
            foreach (var originMesh in loaded.Meshes)
            {
                foreach (var prim in originMesh.Primitives)
                {
                    var attributes = new Dictionary<string, int>();
                    foreach (var kvp in prim.Attributes)
                    {
                        attributes[kvp.Key] = kvp.Value + accessorIncrement;
                    }
                    prim.Attributes = attributes;
                    prim.Indices = prim.Indices + accessorIncrement;
                    if (shouldAddMaterials)
                    {
                        prim.Material = prim.Material + materialIncrement;
                    }
                    else
                    {
                        prim.Material = 0;  // This assumes that the default material is at index 0
                    }

                }
                meshes.Add(originMesh);
                meshIndices.Add(meshes.Count - 1);
            }

            return meshIndices;
        }

        private static void AddMaterials(List<glTFLoader.Schema.Material> materials, Gltf loaded, int textureIncrement)
        {
            foreach (var originMaterial in loaded.Materials)
            {
                if (originMaterial.EmissiveTexture != null)
                {
                    originMaterial.EmissiveTexture.Index = originMaterial.EmissiveTexture.Index + textureIncrement;
                }
                if (originMaterial.NormalTexture != null)
                {
                    originMaterial.NormalTexture.Index = originMaterial.NormalTexture.Index + textureIncrement;
                }
                if (originMaterial.OcclusionTexture != null)
                {
                    originMaterial.OcclusionTexture.Index = originMaterial.OcclusionTexture.Index + textureIncrement;
                }
                if (originMaterial.PbrMetallicRoughness != null)
                {
                    if (originMaterial.PbrMetallicRoughness.MetallicRoughnessTexture != null)
                    {
                        originMaterial.PbrMetallicRoughness.MetallicRoughnessTexture.Index = originMaterial.PbrMetallicRoughness.MetallicRoughnessTexture.Index + textureIncrement;
                    }
                    if (originMaterial.PbrMetallicRoughness.BaseColorTexture != null)
                    {
                        originMaterial.PbrMetallicRoughness.BaseColorTexture.Index = originMaterial.PbrMetallicRoughness.BaseColorTexture.Index + textureIncrement;
                    }
                }

                materials.Add(originMaterial);
            }
        }
    }
}