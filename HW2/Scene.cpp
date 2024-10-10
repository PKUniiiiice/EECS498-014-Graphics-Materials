#include "Scene.h"
#include "Config.h"
#include <iostream>
#include <filesystem>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"


Vec3 Scene::trace(const Ray &ray, int bouncesLeft, bool discardEmission) {
    if constexpr(DEBUG) {
        assert (ray.isNormalized());
    }
    /*
    //task 1
    Intersection inter = getIntersection(ray);

    if (!inter.happened) {
        return {};
    }
    else {
        Vec3 diffuseColor = inter.getDiffuseColor();
        return diffuseColor;
    }
    */
    /*
    //task 2/3
    // ray is camera ray
    if (bouncesLeft < 0) return {};

    Intersection inter = getIntersection(ray);
    if (!inter.happened) {
        return {};
    }
    else {
        Vec3 wi_dir = Random::randomHemisphereDirection(inter.getNormal());
        Ray secondRay = Ray{inter.pos, wi_dir};
        Vec3 brdf = inter.calcBRDF(-secondRay.dir, -ray.dir);

        float cosineTerm = secondRay.dir.dot(inter.getNormal());
        // get Li
        Intersection inter_Li = getIntersection(secondRay);

        if (!inter_Li.happened){
            return inter.getEmission();
        }
        else{
            //Vec3 Li = inter_Li.getEmission();
            Vec3 Li = trace(secondRay, bouncesLeft-1, false);
            Vec3 Lo = inter.getEmission() + (2*PI*cosineTerm)*Li*brdf;
            return Lo;
        }
    }
    */
    //task 4
    if (bouncesLeft < 0) return {};
    Intersection inter = getIntersection(ray);
    if (!inter.happened){
        return {};
    }
    else{
        Vec3 L_indirect = {0.0f, 0.0f, 0.0f};
        Vec3 L_direct = {0.0f, 0.0f, 0.0f};
        // indirect radiance
        Vec3 wi_dir = Random::cosWeightedHemisphere(inter.getNormal());
        Ray secondRay = Ray{inter.pos, wi_dir};
        Vec3 brdf = inter.calcBRDF(-secondRay.dir, -ray.dir);
        Intersection inter_Li = getIntersection(secondRay);
        if (inter_Li.happened){
            L_indirect = PI*brdf*trace(secondRay, bouncesLeft-1, true);
        }
        //direct radiance
        float pdfLightSample = 1 / lightArea;
        Intersection lightSample = sampleLight();
        Vec3 lightDir = lightSample.pos - inter.pos;
        float distanceToLight = lightDir.getLength();
        lightDir.normalize();
        //shawdow ray
        Ray rayToLight = {inter.pos, lightDir};
        Intersection shawdow_inter = getIntersection(rayToLight);
        if ((shawdow_inter.pos-lightSample.pos).getLength()<1e-3){
            //not block
            Vec3 brdf_direct = inter.calcBRDF(-lightDir, -ray.dir);
            float cos_n_wi = lightDir.dot(inter.getNormal());
            float cos_np_wi = -lightSample.getNormal().dot(lightDir);
            L_direct = 1/(pdfLightSample*distanceToLight*distanceToLight) * (lightSample.getEmission() * brdf_direct * cos_n_wi * cos_np_wi);
        }
        //total Li
        Vec3 Li = L_indirect + L_direct;
        Vec3 Lo =  {0.0f, 0.0f, 0.0f};
        if (discardEmission){
            Lo = Li;
        }
        else{
            Lo = inter.getEmission() + Li;
        }
        return Lo;

    }
}

tinyobj::ObjReader Scene::reader {};

void Scene::addObjects(std::string_view modelPath, std::string_view searchPath) {
    tinyobj::ObjReaderConfig config;
    config.mtl_search_path = searchPath;
    if (!reader.ParseFromFile(std::string(modelPath), config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
            std::filesystem::path relative(modelPath);
            std::cerr << "Reading file " << std::filesystem::absolute(relative) << " error. File may be malformed or not exist.\n";
        }
        exit(1);
    }

    auto& attrib = reader.GetAttrib();
    auto& shapes = reader.GetShapes();
    auto& materials = reader.GetMaterials();

    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        auto* object = new Object();
        object->name = shapes[s].name;
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);
            std::vector<Vec3> positions;
            // Loop over vertices in the face.
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
                tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
                tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

                positions.push_back({vx, vy, vz});
            } // per-vertex
            index_offset += fv;
            Mesh mesh {positions[0], positions[1], positions[2]};
            object->area += mesh.area;
            object->meshes.push_back(std::move(mesh));
        } // per-face
        object->constructBoundingBox();
        // we assume each object uses only a single material for all meshes
        auto materialId = shapes[s].mesh.material_ids[0];
        auto& material = materials[materialId];
        object->kd = Vec3 {
            material.diffuse[0],
            material.diffuse[1],
            material.diffuse[2],
        };
        if (material.emission[0] + 
            material.emission[1] + 
            material.emission[2] > 0) { // is light
            object->ke = Vec3 {
                material.emission[0], 
                material.emission[1],
                material.emission[2]
            };
            object->hasEmission = true;
            lights.push_back(object);
            lightArea += object->area;
        }
        objects.push_back(object);
    } // per-shape
}

void Scene::constructBVH() {
    assert (!objects.empty());
    bvh.root = BVH::build(objects);
}

Intersection Scene::getIntersection(const Ray &ray) {
    assert (bvh.root);
    return bvh.root->intersect(ray);
}

Intersection Scene::sampleLight() const {
    assert (lights.size() == 1 && "Currently only support a single light object");
    assert (lightArea > 0.0f);
    Intersection inter;
    return lights[0]->sample();
}

Scene::~Scene() {
    for (auto obj : objects) {
        delete obj;
    }
}
