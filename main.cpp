#include "assignment.hpp"
#include <chrono>
#include <time.h>
#include <atomic>


const int Slabs_num{ 100 };   // has to be a perfect square
int remain_slabs{ Slabs_num }; 

// ******* Function Member Implementation *******

void multi_threads(std::shared_ptr<World>& world, const Pinhole pinhole);
void render_all(std::shared_ptr<World>& world, const Pinhole pinhole, std::shared_ptr<ShadeRec>& trace_data);
void render_slab(std::shared_ptr<Slab> slab, std::shared_ptr<World>& world, const Pinhole pinhole, std::shared_ptr<ShadeRec>& trace_data);
void render_edge(std::shared_ptr<World>& world, const Pinhole pinhole);
void slab_process();

// ***** Instance *****
Matrix Instance::forward_matrix;	
void Instance::compute_bounding_box(void) {
    BBox b = bbox;
    int size = 8;
    atlas::math::Point p[8];

    p[0] = atlas::math::Point(b.x0, b.y0, b.z0);
    p[1] = atlas::math::Point(b.x1, b.y0, b.z0);
    p[2] = atlas::math::Point(b.x0, b.y1, b.z0);
    p[3] = atlas::math::Point(b.x1, b.y1, b.z0);
    p[4] = atlas::math::Point(b.x0, b.y0, b.z1);
    p[5] = atlas::math::Point(b.x1, b.y0, b.z1);
    p[6] = atlas::math::Point(b.x0, b.y1, b.z1);
    p[7] = atlas::math::Point(b.x1, b.y1, b.z1);

    //	box = new Box(atlas::math::Point(b.x0, b.y0, b.z0), atlas::math::Point(b.x1, b.y1, b.z1));
    //	box->set_material(new Phong());

    double minX = kHugeValue, minY = kHugeValue, minZ = kHugeValue;
    double maxX = -kHugeValue, maxY = -kHugeValue, maxZ = -kHugeValue;

    for (int j = 0; j < size; j++)
    {
        // order shouldn't matter here, right?
        p[j] = forward_matrix * p[j];

        minX = std::min(minX, (double)p[j].x);
        minY = std::min(minY, (double)p[j].y);
        minZ = std::min(minZ, (double)p[j].z);
        maxX = std::max(maxX, (double)p[j].x);
        maxY = std::max(maxY, (double)p[j].y);
        maxZ = std::max(maxZ, (double)p[j].z);
    }

    bbox = BBox(atlas::math::Point(minX, minY, minZ), atlas::math::Point(maxX, maxY, maxZ));

    //	box = new Box(atlas::math::Point(minX, minY, minZ), atlas::math::Point(maxX, maxY, maxZ));
    //	box->set_material(new Phong());
    //	Box* bot_ptr = box;

        // reset forward_matrix for next instance to use
    forward_matrix.set_identity();
}




// ***** Utility *****

/*
ViewPlane::ViewPlane(void)
    : hres(400),
    vres(400),
    image_hres(400),
    image_vres(400),
    s(1.0),
    gamma(1.0),
    inv_gamma(1.0),
    show_out_of_gamut(false),
    sampler_ptr(NULL),
    max_depth(0)
{}

ViewPlane::ViewPlane(const ViewPlane& vp)
    : hres(vp.hres),
    vres(vp.vres),
    image_hres(vp.hres),
    image_vres(vp.vres),
    s(vp.s),
    gamma(vp.gamma),
    inv_gamma(vp.inv_gamma),
    show_out_of_gamut(vp.show_out_of_gamut),
    sampler_ptr(vp.sampler_ptr),
    num_samples(vp.num_samples),
    max_depth(vp.max_depth)
{}

ViewPlane& ViewPlane::operator= (const ViewPlane& rhs) {
    if (this == &rhs)
        return (*this);

    hres = rhs.hres;
    vres = rhs.vres;
    image_hres = rhs.hres;
    image_vres = rhs.vres;
    s = rhs.s;
    gamma = rhs.gamma;
    inv_gamma = rhs.inv_gamma;
    show_out_of_gamut = rhs.show_out_of_gamut;
    num_samples = rhs.num_samples;
    sampler_ptr = rhs.sampler_ptr;
    max_depth = rhs.max_depth;

    return (*this);
}

ViewPlane::~ViewPlane(void) {}

void ViewPlane::set_sampler(Sampler* sp) {
    if (sampler_ptr) {
        delete sampler_ptr;
        sampler_ptr = NULL;
    }

    num_samples = sp->getNumSamples();
    sampler_ptr = sp;
}

void ViewPlane::set_samples(const int n) {
    num_samples = n;
    if (sampler_ptr)
    {
        delete sampler_ptr;
        sampler_ptr = NULL;
    }
    if (num_samples > 1)
    {
        sampler_ptr = new MultiJittered(num_samples);
    }
    else
    {
        sampler_ptr = new Regular(1);
    }
}
*/

// ***** World *****

// orthographic viewing 
void World::render_scene() {
    printf("start render_scene\n");
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;
    Ray<atlas::math::Vector> ray{};
    int hres = (int) width;
    int vres = (int) height;
    float zw = 100.0;			// hardwired in
    int	n = (int)sqrt((float)sampler->getNumSamples());

    Point sp, pp;	
    ray.d = Point(0, 0, -1);
    for (int r = 0; r < vres; r++) {
        for (int c = 0; c <= hres; c++) {
            Colour	pixel_color{ 0,0,0 };
            for (int p = 0; p < n; p++)
            {
                for (int q = 0; q < n; q++)
                {
                    sp = sampler->sampleUnitSquare();
                    pp.x = (float)(s * (c - 0.5 * hres + sp.x));
                    pp.y = (float)(s * (r - 0.5 * vres + sp.y));
                    ray.o = Point(pp.x, pp.y, zw);

                 ////////////////////////////   
                    ShadeRec trace_data(*this);
                    trace_data.t = std::numeric_limits<float>::max();
                    bool hit{};
                    for (auto obj : scene)
                    {
                        double t = 0.0f;
                        hit |= obj->hit(ray,t, trace_data);
                    }
                    if (hit)
                        pixel_color += trace_data.material->global_shade(trace_data);
                 //////////////////////////


                    //pixel_color += tracer_ptr->trace_ray(ray);
                    //printf("aft trace_ray\n");
                }
            }
            float avg{ 1.0f / sampler->getNumSamples() };
            pixel_color *= avg;	// Average colors
            //display_pixel(r, c, pixel_color);
            image.push_back(pixel_color);
        }
        process_indicator(r, vres);
        

    }
}

ShadeRec World::hit_objects(atlas::math::Ray<atlas::math::Vector> const& ray) {
    ShadeRec sr(*this);
   // printf("204 World.ambient:(%f,%f,%f) \n", sr.world.ambient->L(sr).r, sr.world.ambient->L(sr).g, sr.world.ambient->L(sr).b);
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;
    double t;
    Vector normal = { 0,0,0 };
    Point local_hit_point = { 0,0,0 };
    float tmin = (float)1.0E10;
    //mutex.lock();
    int	num_objects = (int) scene.size();
    for (int j = 0; j < num_objects; j++) {
        if (scene[j]->hit(ray, t, sr) && (t < tmin)) {
            sr.hit_an_object = true;
            tmin = (float)t;
            sr.material = scene[j]->getMaterial();
            sr.local_hit_point = ray.o + (float)t * ray.d;
            normal = sr.normal;
            local_hit_point = sr.local_hit_point;
        }
    }
    //mutex.unlock();
    if (sr.hit_an_object) {
        sr.t = tmin;
        sr.normal = normal;
        sr.local_hit_point = local_hit_point;
    }
    return (sr);
}

ShadeRec World::hit_bare_bones_objects(atlas::math::Ray<atlas::math::Vector> const& ray) {
    ShadeRec sr(*this);
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;
    double t;
    float tmin = (float)1.0E10;
    int num_objects = (int)scene.size();

    for (int j = 0; j < num_objects; j++)
        if (scene[j]->hit(ray, t, sr) && (t < tmin)) {
            sr.hit_an_object = true;
            tmin = (float)t;
            sr.color = scene[j]->getColour();
        }
    return (sr);
}



// ***** BBox *****
BBox::BBox(void)
    : x0(-1), x1(1), y0(-1), y1(1), z0(-1), z1(1)
{}

BBox::BBox(const float _x0, const float _x1,
    const float _y0, const float _y1,
    const float _z0, const float _z1)
    : x0(_x0), x1(_x1), y0(_y0), y1(_y1), z0(_z0), z1(_z1)
{}

BBox::BBox(const atlas::math::Point p0, const atlas::math::Point p1)
    : x0(p0.x), x1(p1.x), y0(p0.y), y1(p1.y), z0(p0.z), z1(p1.z)
{}

BBox::BBox(const BBox& bbox)
    : x0(bbox.x0), x1(bbox.x1), y0(bbox.y0), y1(bbox.y1), z0(bbox.z0), z1(bbox.z1)
{}

BBox& BBox::operator=(const BBox& rhs) {
    if (this == &rhs)
        return (*this);
    x0 = rhs.x0; x1 = rhs.x1; y0 = rhs.y0; y1 = rhs.y1; z0 = rhs.z0; z1 = rhs.z1;
    return (*this);
}

BBox* BBox::clone(void) const {
    return (new BBox(*this));
}

BBox::~BBox(void) {}

bool BBox::hit(atlas::math::Ray<atlas::math::Vector> const& ray) const {
    float ox = ray.o.x;
    float oy = ray.o.y;
    float oz = ray.o.z;
    float dx = ray.d.x;
    float dy = ray.d.y;
    float dz = ray.d.z;
    float tx_min, ty_min, tz_min;
    float tx_max, ty_max, tz_max;
    float a = (float)( 1.0 / dx);
    if (a >= 0) {
        tx_min = (x0 - ox) * a;
        tx_max = (x1 - ox) * a;
    }
    else {
        tx_min = (x1 - ox) * a;
        tx_max = (x0 - ox) * a;
    }
    float b = (float)(1.0 / dy);
    if (b >= 0) {
        ty_min = (y0 - oy) * b;
        ty_max = (y1 - oy) * b;
    }
    else {
        ty_min = (y1 - oy) * b;
        ty_max = (y0 - oy) * b;
    }

    float c = (float)(1.0 / dz);
    if (c >= 0) {
        tz_min = (z0 - oz) * c;
        tz_max = (z1 - oz) * c;
    }
    else {
        tz_min = (z1 - oz) * c;
        tz_max = (z0 - oz) * c;
    }
    float t0, t1;
    // find largest entering t value
    if (tx_min > ty_min)
        t0 = tx_min;
    else
        t0 = ty_min;
    if (tz_min > t0)
        t0 = tz_min;
    // find smallest exiting t value
    if (tx_max < ty_max)
        t1 = tx_max;
    else {
        t1 = ty_max;
    }
    if (tz_max < t1)
        t1 = tz_max;
    return (t0 < t1 && t1 > kEpsilon);

}

bool
BBox::inside(const atlas::math::Point& p) const {
    return ((p.x > x0&& p.x < x1) && (p.y > y0&& p.y < y1) && (p.z > z0&& p.z < z1));
};
 

// ***** Camera function members *****

// ***** Pinhole function members *****
Pinhole::Pinhole() : Camera(), mDistance{ 500.0f }, mZoom{ 1.0f }
{}

void Pinhole::setDistance(float distance)
{
    mDistance = distance;
}

void Pinhole::setZoom(float zoom)
{
    mZoom = zoom;
}

atlas::math::Vector Pinhole::rayDirection(atlas::math::Point const& p) const
{
    const auto dir = p.x * u + p.y * v - mDistance * w;
    return glm::normalize(dir);
}


void Pinhole::renderScene(std::shared_ptr<World> world)
{
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;
    Point samplePoint{}, pixelPoint{};
    Colour L;
    //ViewPlane vp(world->vp);
    Ray<atlas::math::Vector> ray{};
    int depth = 0; //depth start from 0;	
    world->s /= mZoom;
    ray.o = mEye;

    for (int r = 0; r < world->height; r++) {	// up 
        for (int c = 0; c < world->width; c++) {
            L = black;
            for (int j = 0; j < world->sampler->getNumSamples(); j++) {
                samplePoint = world->sampler->sampleUnitSquare();
                pixelPoint.x = (float)(world->s * (c - 0.5 * world->width + samplePoint.x));
                pixelPoint.y = -(float)(world->s * (r - 0.5 * world->height + samplePoint.y));
                ray.d = rayDirection(pixelPoint);
                
        
                //Colour temp = world->tracer_ptr->trace_ray(ray, 0);
                //printf("Pinhole :(%f,%f,%f)\n", temp.r, temp.g, temp.b);
                //L += temp;

                L += world->tracer_ptr->trace_ray(ray, depth);

                //printf("L is (%f,%f,%f)\n", L.x, L.y, L.z);
                //////////////////////////////
                //ShadeRec trace_data(*world);
                //trace_data.world = *world;
                //trace_data.t = std::numeric_limits<float>::max();
                //bool hit{};
                //for (auto obj : world->scene)
                //{
                //    double t = 0.0f;
                //    hit |= obj->hit(ray, t, trace_data);
                //}
                //if (hit) {
                //    L += trace_data.material->shade(trace_data);
                //    //L += trace_data.material->path_shade(trace_data);
                //}
                //////////////////////////////
            }   
                
            float avg{ 1.0f / world->sampler->getNumSamples() };
            L *= avg;
            L *= 1.0;       //exposure_time;

            colour_handling(L);
            //out-of-gamut control

            world->image.push_back(L);

            //world->display_pixel(r, c, L);
        }
        process_indicator(r,(int)world->height);
    }
}

//void Pinhole::renderScene(std::shared_ptr<World> world) const
//{
//    using atlas::math::Point;
//    using atlas::math::Ray;
//    using atlas::math::Vector;
//    Point samplePoint{}, pixelPoint{};
//    Ray<atlas::math::Vector> ray{};
//    ray.o = mEye;
//    float avg{ 1.0f / world->sampler->getNumSamples() };
//    for (int r{ 0 }; r < world->height; ++r)
//    {
//        for (int c{ 0 }; c < world->width; ++c)
//        {
//            Colour pixelAverage{ 0, 0, 0 };
//            for (int j = 0; j < world->sampler->getNumSamples(); ++j)
//            {
//                ShadeRec trace_data{};
//                trace_data.world = world;
//                trace_data.t = std::numeric_limits<float>::max();
//                samplePoint = world->sampler->sampleUnitSquare();
//                pixelPoint.x = c - 0.5f * world->width + samplePoint.x;
//                pixelPoint.y = -(r - 0.5f * world->height + samplePoint.y);
//                ray.d = rayDirection(pixelPoint);
//                //L += world->tracer_ptr->trace_ray(ray, depth);
//                bool hit{};
//                for (auto obj : world->scene)
//                {
//                    float t = 0.0f;
//                    hit |= obj->hit(ray,t, trace_data);
//                }
//                if (hit)
//                {
//                    //pixelAverage += trace_data.material->shade(trace_data);
//                    pixelAverage += trace_data.material->area_light_shade(trace_data);
//                }
//            }
//            pixelAverage.r *= avg;
//            pixelAverage.g *= avg;
//            pixelAverage.b *= avg;
//            colour_handling(pixelAverage);
//            //show out-of-gamut
//            if (pixelAverage.r > 1 || pixelAverage.g > 1 || pixelAverage.b > 1) {
//                pixelAverage.r = 1.0; pixelAverage.g = 1.0;  pixelAverage.b = 0;
//            }
//
//            world->image.push_back(pixelAverage);
//        }
//    }
//}



// ***** Shape function members *****


// ***** Sphere function members *****




// ***** Matte function members *****

Matte::Matte(float kd, float ka, Colour color) : Matte{}
{
    setDiffuseReflection(kd);
    setAmbientReflection(ka);
    setDiffuseColour(color);
}

void Matte::setDiffuseReflection(float k)
{
    mDiffuseBRDF->setDiffuseReflection(k);
}

void Matte::setAmbientReflection(float k)
{
    mAmbientBRDF->setDiffuseReflection(k);
}

void Matte::setDiffuseColour(Colour colour)
{
    mDiffuseBRDF->setDiffuseColour(colour);
    mAmbientBRDF->setDiffuseColour(colour);
}

Colour Matte::shade(ShadeRec& sr)
{
    using atlas::math::Ray;
    using atlas::math::Vector;
    Vector wo = -sr.ray.o;
    Colour L = mAmbientBRDF->rho(sr, wo) * sr.world.ambient->L(sr); 
    size_t numLights = sr.world.lights.size();
    for (size_t i{ 0 }; i < numLights; ++i)
    {
        Vector wi = sr.world.lights[i]->getDirection(sr);
        float nDotWi = glm::dot(sr.normal, wi);
        if (nDotWi > 0.0f)
        {
            L += mDiffuseBRDF->fn(sr, wo, wi) * sr.world.lights[i]->L(sr) * nDotWi;
        }
    }
    return L;
}


Colour Matte::global_shade(ShadeRec& sr) {
    Colour L;

    if (sr.depth == 0)
        L = area_light_shade(sr);

    atlas::math::Vector wi;
    atlas::math::Vector wo = -sr.ray.d;
    float pdf;
    Colour f = mDiffuseBRDF->sample_f(sr, wi, wo, pdf);
    float ndotwi = glm::dot(sr.normal , wi);
    atlas::math::Ray<atlas::math::Vector> reflected_ray(sr.local_hit_point, wi);
    L += f * sr.world.tracer_ptr->trace_ray(reflected_ray, sr.depth + 1) * ndotwi / pdf;

    return (L);
}





// ***** Directional function members *****
Directional::Directional() : Light(),
ls(1.0),
color(1.0),
direction(0, 1, 0)
{}

Directional::Directional(const Directional& pl) : Light(pl),
ls(pl.ls),
color(pl.color),
direction(pl.direction)
{}

atlas::math::Vector Directional::getDirection([[maybe_unused]] ShadeRec& sr)
{
    return direction;
}



// ***** AreaLight function members *****
FakeSphericalLight::FakeSphericalLight(void)
    : Light(),ls(1.0),color(1.0),location(0),r(1.0)
{}

FakeSphericalLight::FakeSphericalLight(const FakeSphericalLight& pl)
    : Light(pl), ls(pl.ls), color(pl.color), location(pl.location), r(pl.r)
{}

FakeSphericalLight&
FakeSphericalLight::operator= (const FakeSphericalLight& pl) {
    if (this == &pl)
        return (*this);
    Light::operator= (pl);
    ls = pl.ls;
    color = pl.color;
    location = pl.location;
    r = pl.r;
    return (*this);
}


atlas::math::Vector
FakeSphericalLight::getDirection(ShadeRec& sr) {
    atlas::math::Point new_location;
    new_location.x = (float)(location.x + r * (2.0 * rand_float() - 1.0));
    new_location.y = (float)(location.y + r * (2.0 * rand_float() - 1.0));
    new_location.z = (float)(location.z + r * (2.0 * rand_float() - 1.0));

    atlas::math::Vector n = new_location - sr.local_hit_point;
    float length = sqrt(n.x * n.x + n.y * n.y + n.z * n.z);
    n = n / length;
    return(n);
}

bool
FakeSphericalLight::in_shadow(atlas::math::Ray<atlas::math::Vector> const& ray, const ShadeRec& sr) const {
    float t;
    int num_objects = (int)sr.world.scene.size();
    float d = sqrt((location.x - ray.o.x)* (location.x - ray.o.x)+(location.y-ray.o.y)* (location.y - ray.o.y)+(location.z-ray.o.z)* (location.z - ray.o.z));     //distance

    for (int j = 0; j < num_objects; j++)
        if (sr.world.scene[j]->shadow_hit(ray, t) && t < d)
            return (true);

    return (false);
}




// ***** Lambertian function members *****
Lambertian::Lambertian() : BRDF(), mDiffuseColour{0.0}, mDiffuseReflection{0.0}
{}

Lambertian::Lambertian(Colour diffuseColor, float diffuseReflection) :
    mDiffuseColour{ diffuseColor }, mDiffuseReflection{ diffuseReflection }
{}

Colour
Lambertian::fn([[maybe_unused]] ShadeRec const& sr,
    [[maybe_unused]] atlas::math::Vector const& reflected,
    [[maybe_unused]] atlas::math::Vector const& incoming) const
{
    return mDiffuseColour * mDiffuseReflection * glm::one_over_pi<float>();
}

Colour
Lambertian::rho([[maybe_unused]] ShadeRec const& sr,
    [[maybe_unused]] atlas::math::Vector const& reflected) const
{
    return mDiffuseColour * mDiffuseReflection;
}

void Lambertian::setDiffuseReflection(float kd)
{
    mDiffuseReflection = kd;
}

void Lambertian::setDiffuseColour(Colour const& colour)
{
    mDiffuseColour = colour;
}


// ***** Sampler function members *****


void Sampler::setupShuffledIndeces()
{
    mShuffledIndeces.reserve(mNumSamples * mNumSets);
    std::vector<int> indices;
    std::random_device d;
    std::mt19937 generator(d());
    for (int j = 0; j < mNumSamples; ++j)
    {
        indices.push_back(j);
    }
    for (int p = 0; p < mNumSets; ++p)
    {
        std::shuffle(indices.begin(), indices.end(), generator);

        for (int j = 0; j < mNumSamples; ++j)
        {
            mShuffledIndeces.push_back(indices[j]);
        }
    }
}

// ***** Regular function members *****
Regular::Regular(int numSamples, int numSets) : Sampler{ numSamples, numSets }
{
    generateSamples();
}

void Regular::generateSamples()
{
    int n = static_cast<int>(glm::sqrt(static_cast<float>(mNumSamples)));
    for (int j = 0; j < mNumSets; ++j)
    {
        for (int p = 0; p < n; ++p)
        {
            for (int q = 0; q < n; ++q)
            {
                mSamples.push_back(
                    atlas::math::Point{ (q + 0.5f) / n, (p + 0.5f) / n, 0.0f });
            }
        }
    }
}

// ***** Random function members *****
Random::Random(int numSamples, int numSets) : Sampler{ numSamples, numSets }
{
    generateSamples();
}

void Random::generateSamples()
{
    atlas::math::Random<float> engine;
    for (int p = 0; p < mNumSets; ++p)
    {
        for (int q = 0; q < mNumSamples; ++q)
        {
            mSamples.push_back(atlas::math::Point{
                engine.getRandomOne(), engine.getRandomOne(), 0.0f });
        }
    }
}

Jittered::Jittered(int numSamples, int numSets) : Sampler{ numSamples, numSets }
{
    generateSamples();
}

void Jittered::generateSamples()
{
    atlas::math::Random<float> engine;
    for (std::size_t p{ 0 }; p < mNumSets; ++p) {
        for (std::size_t q{ 0 }; q < mNumSamples; ++q) {
            float sampleX = (q + engine.getRandomOne()) / mNumSamples;
            float sampleY = (p + engine.getRandomOne()) / mNumSets;
            mSamples.push_back(atlas::math::Point{ sampleX, sampleY, 0.0f });
        }
    }
}

MultiJittered::MultiJittered(const int num_samples, const int m)
    : Sampler(num_samples, m) {
    generateSamples();
}

void MultiJittered::generateSamples() 
{
    atlas::math::Random<float> engine;
    // num_samples needs to be a perfect square
    int n = (int)sqrt((float)mNumSamples);
    float subcell_width = (float)1.0 / ((float)mNumSamples);
    atlas::math::Point fill_point;
    for (int j = 0; j < mNumSamples * mNumSets; j++)
        mSamples.push_back(fill_point);
    for (int p = 0; p < mNumSets; p++)
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                mSamples[i * n + j + p * mNumSamples].x = (i * n + j) * subcell_width + engine.getRandomRange(0, subcell_width);
                mSamples[i * n + j + p * mNumSamples].y = (j * n + i) * subcell_width + engine.getRandomRange(0, subcell_width);
            }
    atlas::math::Random<int> engine2;
    // shuffle x coordinates
    for (int p = 0; p < mNumSets; p++)
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                int k = engine2.getRandomRange(j, n - 1);
                float t = mSamples[i * n + j + p * mNumSamples].x;
                mSamples[i * n + j + p * mNumSamples].x = mSamples[i * n + k + p * mNumSamples].x;
                mSamples[i * n + k + p * mNumSamples].x = t;
            }
    // shuffle y coordinates
    for (int p = 0; p < mNumSets; p++)
        for (int i = 0; i < n; i++)
            for (int j = 0; j < n; j++) {
                int k = engine2.getRandomRange(j, n - 1);
                float t = mSamples[j * n + i + p * mNumSamples].y;
                mSamples[j * n + i + p * mNumSamples].y = mSamples[k * n + i + p * mNumSamples].y;
                mSamples[k * n + i + p * mNumSamples].y = t;
            }
}
// ***** Transparent function members *****

Transparent::Transparent(void)
    : reflective_brdf(new PerfectSpecular),
    specular_btdf(new PerfectTransmitter) {
    //	reflective_brdf->set_cr(white);
}

Transparent::Transparent(const Transparent& trans)
    : reflective_brdf(trans.reflective_brdf),
    specular_btdf(trans.specular_btdf) {}

//Transparent*
//Transparent::clone(void) const {
//    return (new Transparent(*this));
//}

Transparent&
Transparent::operator= (const Transparent& trans) {
    if (this == &trans)
        return (*this);

    if (reflective_brdf)
    {
        delete reflective_brdf;
        reflective_brdf = NULL;
    }

    if (specular_btdf) {
        delete specular_btdf;
        specular_btdf = NULL;
    }

    if (trans.reflective_brdf)
        reflective_brdf = trans.reflective_brdf;

    if (trans.specular_btdf)
        specular_btdf = trans.specular_btdf;

    return (*this);
}

Colour
Transparent::shade(ShadeRec& sr) {
    Colour L(Phong::shade(sr));

    atlas::math::Vector wo = -sr.ray.d;
    atlas::math::Vector wi;
    Colour fr = reflective_brdf->sample_f(sr, wi, wo);
    atlas::math::Ray<atlas::math::Vector> reflected_ray(sr.hit_point, wi);

    if (specular_btdf->tir(sr))
        L += sr.world.tracer_ptr->trace_ray(reflected_ray, sr.depth + 1);
    else {
        atlas::math::Vector wt;
        Colour ft = specular_btdf->sample_f(sr, wo, wt);
        atlas::math::Ray<atlas::math::Vector> transmitted_ray(sr.hit_point, wt);

        L += fr * sr.world.tracer_ptr->trace_ray(reflected_ray, sr.depth + 1) * fabs(glm::dot(sr.normal , wi));
        L += ft * sr.world.tracer_ptr->trace_ray(transmitted_ray, sr.depth + 1) * fabs(glm::dot(sr.normal , wt));
    }

    return (L);
}

Colour Transparent::area_light_shade(ShadeRec& sr) {
    Colour L(Phong::shade(sr));

    atlas::math::Vector wo = -sr.ray.d;
    atlas::math::Vector wi;
    Colour fr = reflective_brdf->sample_f(sr, wi, wo);
    atlas::math::Ray<atlas::math::Vector> reflected_ray;
    reflected_ray.o = sr.local_hit_point;
    reflected_ray.d = wi;

    if (specular_btdf->tir(sr))
        L += sr.world.tracer_ptr->trace_ray(reflected_ray, sr.depth + 1);
    else {
        atlas::math::Vector wt;
        Colour ft = specular_btdf->sample_f(sr, wo, wt);
        atlas::math::Ray<atlas::math::Vector> transmitted_ray;
        transmitted_ray.o = sr.local_hit_point;
        transmitted_ray.d = wt;

        //L += fr * sr.world.tracer_ptr->trace_ray(reflected_ray, sr.depth + 1) * fabs(glm::dot(sr.normal, wi));
        Colour temp1 = fr * sr.world.tracer_ptr->trace_ray(reflected_ray, sr.depth + 1);
        float temp2 = fabs(glm::dot(sr.normal, wi));
        L += temp1 * temp2;
        Colour temp3 = ft * sr.world.tracer_ptr->trace_ray(transmitted_ray, sr.depth + 1);
        float temp4 = fabs(glm::dot(sr.normal, wt));
        L += temp3 * temp4;
    }
    return (L);

}

Transparent::~Transparent(void) {
    if (reflective_brdf)
    {
        delete reflective_brdf;
        reflective_brdf = NULL;
    }

    if (specular_btdf) {
        delete specular_btdf;
        specular_btdf = NULL;
    }
}

//// --------------------------------------------- default constructor
//
//Point3D::Point3D()
//    :x(0), y(0), z(0)
//{}
//
//
//// --------------------------------------------- constructor
//
//Point3D::Point3D(const double a)
//    :x(a), y(a), z(a)
//{}
//
//// --------------------------------------------- constructor
//
//Point3D::Point3D(const double a, const double b, const double c)
//    : x(a), y(b), z(c)
//{}
//
//
//// --------------------------------------------- copy constructor
//
//Point3D::Point3D(const Point3D& p)
//    :x(p.x), y(p.y), z(p.z)
//{}
//
//
//// --------------------------------------------- destructor
//
//Point3D::~Point3D()
//{}
//
//
//// --------------------------------------------- assignment operator
//
//Point3D&
//Point3D::operator= (const Point3D& rhs) {
//
//    if (this == &rhs)
//        return (*this);
//
//    x = rhs.x; y = rhs.y; z = rhs.z;
//
//    return (*this);
//}
//
//
//
//// --------------------------------------------- distance
//// distance between two points
//
//double
//Point3D::distance(const Point3D& p) const {
//    return (sqrt((x - p.x) * (x - p.x)
//        + (y - p.y) * (y - p.y)
//        + (z - p.z) * (z - p.z)));
//}
//
//
//// non-member function
//
//// --------------------------------------------- operator*
//// multiplication by a matrix on the left
//
//Point3D
//operator* (const Matrix& mat, const Point3D& p) {
//    return (Point3D(mat.m[0][0] * p.x + mat.m[0][1] * p.y + mat.m[0][2] * p.z + mat.m[0][3],
//        mat.m[1][0] * p.x + mat.m[1][1] * p.y + mat.m[1][2] * p.z + mat.m[1][3],
//        mat.m[2][0] * p.x + mat.m[2][1] * p.y + mat.m[2][2] * p.z + mat.m[2][3]));
//}
//
//
//
//
////**********Noise *************
//
//const unsigned char LatticeNoise::permutation_table[kTableSize] =
//{
//
//    225,155,210,108,175,199,221,144,203,116, 70,213, 69,158, 33,252,
//
//    5, 82,173,133,222,139,174, 27,  9, 71, 90,246, 75,130, 91,191,
//
//    169,138,  2,151,194,235, 81,  7, 25,113,228,159,205,253,134,142,
//
//    248, 65,224,217, 22,121,229, 63, 89,103, 96,104,156, 17,201,129,
//
//    36,  8,165,110,237,117,231, 56,132,211,152, 20,181,111,239,218,
//
//    170,163, 51,172,157, 47, 80,212,176,250, 87, 49, 99,242,136,189,
//
//    162,115, 44, 43,124, 94,150, 16,141,247, 32, 10,198,223,255, 72,
//
//    53,131, 84, 57,220,197, 58, 50,208, 11,241, 28,  3,192, 62,202,
//
//    18,215,153, 24, 76, 41, 15,179, 39, 46, 55,  6,128,167, 23,188,
//
//    106, 34,187,140,164, 73,112,182,244,195,227, 13, 35, 77,196,185,
//
//    26,200,226,119, 31,123,168,125,249, 68,183,230,177,135,160,180,
//
//    12,  1,243,148,102,166, 38,238,251, 37,240,126, 64, 74,161, 40,
//
//    184,149,171,178,101, 66, 29, 59,146, 61,254,107, 42, 86,154,  4,
//
//    236,232,120, 21,233,209, 45, 98,193,114, 78, 19,206, 14,118,127,
//
//    48, 79,147, 85, 30,207,219, 54, 88,234,190,122, 95, 67,143,109,
//
//    137,214,145, 93, 92,100,245,  0,216,186, 60, 83,105, 97,204, 52
//
//};
//
//LatticeNoise::LatticeNoise(void)
//
//    : num_octaves(1),
//
//    lacunarity(2.0),
//
//    gain(0.5)
//
//{
//
//    init_value_table(seed_value);
//
//    init_vector_table(seed_value);
//
//    compute_fbm_bounds();
//
//}
//
//
//
//
//
////---------------------------------------------------------------------------------------- constructor
//
//
//
//LatticeNoise::LatticeNoise(int _num_octaves)
//
//    : num_octaves(_num_octaves),
//
//    lacunarity(2.0),
//
//    gain(0.5)
//
//{
//
//    init_value_table(seed_value);
//
//    init_vector_table(seed_value);
//
//    compute_fbm_bounds();
//
//}
//
//
//
//
//
////---------------------------------------------------------------------------------------- constructor
//
//
//
//LatticeNoise::LatticeNoise(int _num_octaves, float _lacunarity, float _gain)
//
//    : num_octaves(_num_octaves),
//
//    lacunarity(_lacunarity),
//
//    gain(_gain) {
//
//    init_value_table(seed_value);
//
//    init_vector_table(seed_value);
//
//    compute_fbm_bounds();
//
//}
//
//
//
//
//
////---------------------------------------------------------------------------------------- copy constructor
//
//
//
//LatticeNoise::LatticeNoise(const LatticeNoise& ln)
//
//    : num_octaves(ln.num_octaves),
//
//    lacunarity(ln.lacunarity),
//
//    gain(ln.gain)
//
//{
//
//    init_value_table(seed_value);
//
//    init_vector_table(seed_value);
//
//    compute_fbm_bounds();
//
//}
//
//
//
//
//
////---------------------------------------------------------------------------------------- assignment operator
//
//
//
//// This doesn't assign value_table and vector_table, but these should be the same for
//
//// all noise objects.
//
//
//
//LatticeNoise&
//
//LatticeNoise::operator= (const LatticeNoise& rhs) {
//
//    if (this == &rhs)
//
//        return (*this);
//
//
//
//    num_octaves = rhs.num_octaves;
//
//    lacunarity = rhs.lacunarity;
//
//    gain = rhs.gain;
//
//
//
//    return (*this);
//
//}
//
//
//
//LatticeNoise::~LatticeNoise(void) {}
//
//
//void LatticeNoise::init_value_table(int seed_values) {
//
//    srand(seed_values);
//
//    for (int i = 0; i < kTableSize; i++)
//
//        value_table[i] = (float)(1.0 - 2.0 * rand_float());   // In the range [-1, +1]
//
//}
//
//void LatticeNoise::init_vector_table(int seed_values) {
//
//    float r1, r2;
//
//    float x, y, z;
//
//    float r, phi;
//
//
//
//    srand(seed_values);
//
//    MultiJittered* sample_ptr = new MultiJittered(256, 1);  // 256 samples, 1 set
//
//
//
//    for (int j = 0; j < kTableSize; j++) {
//
//        atlas::math::Point sample_point = sample_ptr->sampleUnitSquare();  // on the unit square
//
//        r1 = sample_point.x;
//
//        r2 = sample_point.y;
//
//        z = 1.0f - 2.0f * r1;
//
//        r = (float)(sqrt(1.0 - z * z));
//
//        phi = (float)(2.0* PI * r2);
//
//        x = r * cos(phi);
//
//        y = r * sin(phi);
//
//        double lengtht = sqrt(x * x + y * y + z * z);
//        x /= (float)lengtht; y /= (float)lengtht; z /= (float)lengtht;
//
//        vector_table[j] = atlas::math::Vector(x, y, z);
//
//    }
//
//}
//
//
//
//
//
////---------------------------------------------------------------------------------------- set_num_octaves
//
//// we must re-compute the fbm bounds
//
//
//
//void LatticeNoise::set_num_octaves(int _num_octaves) {
//
//    num_octaves = _num_octaves;
//
//    compute_fbm_bounds();
//
//}
//
//
//void LatticeNoise::set_lacunarity(float _lacunarity) {
//
//    lacunarity = _lacunarity;
//
//}
//
//
//
//void LatticeNoise::set_gain(float _gain) {
//
//    gain = _gain;
//
//    compute_fbm_bounds();
//
//}
//
//
//
//void LatticeNoise::compute_fbm_bounds(void) {
//
//    if (gain == 1.0)
//
//        fbm_max = (float)num_octaves;
//
//    else
//
//        fbm_max = (float)((1.0 - pow(gain, num_octaves)) / (1.0 - gain));
//
//
//
//    fbm_min = -fbm_max;
//
//}
//
//
//
//
//
//
//
////---------------------------------------------------------------------------------------- value_fractal_sum
//
//
//
//// This function implements the scalar fractal sum function
//
//// This has gain = 0.5, and lacunarity = 2.0 for self-similarity
//
//
//
//float LatticeNoise::value_fractal_sum(const atlas::math::Point& p) const {
//
//    float 	amplitude = 1.0f;
//
//    float	frequency = 1.0f;
//
//    float 	fractal_sum = 0.0;
//
//
//
//    for (int j = 0; j < num_octaves; j++) {
//
//        fractal_sum += amplitude * value_noise(Point3D((frequency * p).x, (frequency * p).y, (frequency * p).z));
//
//        amplitude *= 0.5f;
//
//        frequency *= 2.0f;
//
//    }
//
//
//
//    fractal_sum = (fractal_sum - fbm_min) / (fbm_max - fbm_min);  // map to [0, 1]
//
//
//
//    return (fractal_sum);
//
//}
//
//
//
//
//
////---------------------------------------------------------------------------------------- vector_fractal_sum
//
//
//
//atlas::math::Vector
//
//LatticeNoise::vector_fractal_sum(const atlas::math::Point& p) const {
//
//    float 		amplitude = 1.0f;
//
//    float		frequency = 1.0f;
//
//    atlas::math::Vector	sum(0.0);
//
//
//
//    for (int j = 0; j < num_octaves; j++) {
//
//        sum += amplitude * vector_noise(Point3D((frequency * p).x, (frequency * p).y, (frequency * p).z));
//
//        amplitude *= 0.5f;
//
//        frequency *= 2.0f;
//
//    }
//
//
//
//    return (sum);
//
//}
//
//
//
//
//
////---------------------------------------------------------------------------------------- value_turbulence
//
//// min turbulence = 0.0, max turbulence = fbm_max
//
//
//
//float
//
//LatticeNoise::value_turbulence(const atlas::math::Point& p) const {
//
//    float 	amplitude = 1.0f;
//
//    float	frequency = 1.0f;
//
//    float 	turbulence = 0.0;
//
//
//
//    for (int j = 0; j < num_octaves; j++) {
//
//        turbulence += amplitude * fabs(value_noise(Point3D((frequency * p).x, (frequency * p).y, (frequency * p).z)));
//
//        //		turbulence	+= amplitude * sqrt(fabs(value_noise(frequency * p)));  // for the thin lines in Figure 30.6 (c) & (d)
//
//        amplitude *= 0.5f;
//
//        frequency *= 2.0f;
//
//    }
//
//
//
//    turbulence /= fbm_max;  // map to [0, 1]
//
//
//
//    return (turbulence);
//
//}
//
//
//float LatticeNoise::value_fbm(const atlas::math::Point& p) const {
//
//    float 	amplitude = 1.0f;
//
//    float	frequency = 1.0f;
//
//    float 	fbm = 0.0;
//
//
//
//    for (int j = 0; j < num_octaves; j++) {
//
//        fbm += amplitude * value_noise(Point3D((frequency * p).x, (frequency * p).y, (frequency * p).z));
//
//        amplitude *= gain;
//
//        frequency *= lacunarity;
//
//    }
//
//
//
//    fbm = (fbm - fbm_min) / (fbm_max - fbm_min);  // map to [0, 1]
//
//
//
//    return (fbm);
//
//}
//
//
//
//
//
////---------------------------------------------------------------------------------------- vector_fbm
//
//
//
//atlas::math::Vector LatticeNoise::vector_fbm(const atlas::math::Point& p) const {
//
//    float 		amplitude = 1.0f;
//
//    float		frequency = 1.0f;
//
//    atlas::math::Vector	sum(0.0);
//
//
//
//    for (int j = 0; j < num_octaves; j++) {
//
//        sum += amplitude * vector_noise(Point3D((frequency * p).x, (frequency * p).y, (frequency * p).z));
//
//        amplitude *= gain;
//
//        frequency *= lacunarity;
//
//    }
//
//
//
//    return (sum);
//
//}
//
//
//CubicNoise::CubicNoise(void)
//
//    : LatticeNoise()
//
//{}
//
//
//
//
//
////------------------------------------------------------------------------- constructor
//
//
//
//CubicNoise::CubicNoise(int octaves)
//
//    : LatticeNoise(octaves)
//
//{}
//
//
//
//
//
////------------------------------------------------------------------------- constructor
//
//
//
//CubicNoise::CubicNoise(int octaves, float lacunarity, float gain)
//
//    : LatticeNoise(octaves, lacunarity, gain)
//
//{}
//
//
//
//
//
////------------------------------------------------------------------------- copy constructor
//
//
//
//CubicNoise::CubicNoise(const CubicNoise& cns)
//
//    : LatticeNoise(cns)
//
//{}
//
//
//
//
//
////------------------------------------------------------------------------- clone
//
//
//
//CubicNoise*
//
//CubicNoise::clone(void) const {
//
//    return (new CubicNoise);
//
//}
//
//
//
//
//
////------------------------------------------------------------------------- destructor
//
//
//
//CubicNoise::~CubicNoise(void) {}
//
//
//
//
//
////------------------------------------------------------------------------- value_noise
//
//
//
//// By clamping here, we don't have to clamp it in the fractal_sum,
//
//// turbulence and fbm functions
//
//// This is based on code from Peachey (2003).
//
//
//
//float
//
//CubicNoise::value_noise(const Point3D& p) const {
//
//    int 	ix, iy, iz;
//
//    float 	fx, fy, fz;
//
//    float 	xknots[4], yknots[4], zknots[4];
//
//
//
//    ix = (int)floor(p.x);
//
//    fx = p.x - ix;
//
//
//
//    iy = (int)floor(p.y);
//
//    fy = p.y - iy;
//
//
//
//    iz = (int)floor(p.z);
//
//    fz = p.z - iz;
//
//
//
//    for (int k = -1; k <= 2; k++) {
//
//        for (int j = -1; j <= 2; j++) {
//
//            for (int i = -1; i <= 2; i++) {
//
//                xknots[i + 1] = value_table[INDEX(ix + i, iy + j, iz + k)];
//
//            }
//
//            yknots[j + 1] = four_knot_spline(fx, xknots);
//
//        }
//
//        zknots[k + 1] = four_knot_spline(fy, yknots);
//
//    }
//
//
//
//    return ((float)clamp(four_knot_spline(fz, zknots), -1.0, 1.0));
//
//}
//
//
//
//
//
//
//
////------------------------------------------------------------------------- vector_noise
//
//// This is based on code from Peachey (2003).
//
//
//
//atlas::math::Vector CubicNoise::vector_noise(const Point3D& p) const {
//
//    int 		ix, iy, iz;
//
//    float 		fx, fy, fz;
//
//    atlas::math::Vector 	xknots[4], yknots[4], zknots[4];
//
//    ix = (int)floor(p.x);
//    fx = p.x - ix;
//
//    iy = (int)floor(p.y);
//    fy = p.y - iy;
//
//    iz = (int)floor(p.z);
//    fz = p.z - iz;
//
//    for (int k = -1; k <= 2; k++) {
//
//        for (int j = -1; j <= 2; j++) {
//
//            for (int i = -1; i <= 2; i++) {
//
//                xknots[i + 1] = vector_table[INDEX(ix + i, iy + j, iz + k)];
//
//            }
//
//            yknots[j + 1] = four_knot_spline(fx, xknots);
//
//        }
//
//        zknots[k + 1] = four_knot_spline(fy, yknots);
//
//    }
//
//    return (four_knot_spline(fz, zknots));
//
//}
//
//
//FBmTextureWrapped::FBmTextureWrapped(void)
//    : Texture(),
//    noise_ptr(NULL),
//    color(0.0),
//    min_value(-0.1f),
//    max_value(1.1f),
//    expansion_number(1.0f)
//
//{}
//
//// ----------------------------------------------------------------constructor
//
//
//
//FBmTextureWrapped::FBmTextureWrapped(LatticeNoise* ln_ptr)
//    : Texture(),
//    noise_ptr(ln_ptr),
//    color(0.0),
//    min_value(-0.1f),
//    max_value(1.1f),
//    expansion_number(1.0f)
//
//{}
//
//
//FBmTextureWrapped::FBmTextureWrapped(const FBmTextureWrapped& ft)
//    : Texture(ft),
//    noise_ptr(ft.noise_ptr),
//    color(ft.color),
//    min_value(ft.min_value),
//    max_value(ft.max_value),
//    expansion_number(ft.expansion_number)
//
//{}
//
//FBmTextureWrapped* FBmTextureWrapped::clone(void) const {
//    return (new FBmTextureWrapped(*this));
//}
//
//FBmTextureWrapped::~FBmTextureWrapped(void) {}
//
//
//
//
//
//


int main()
{
    // Your code here.
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;


    std::shared_ptr<World> world{ std::make_shared<World>() };
    // provide world data
    world->width = 1000;            //has to be eaqual to height    
    world->height = 1000;            
    world->background = { 0, 0, 0 };
    //world->sampler = std::make_shared<MultiJittered>(49, 83);     //
    //world->sampler = std::make_shared<MultiJittered>(16);         //
    //world->sampler = std::make_shared<Regular>(1);        //
    std::shared_ptr<MultiJittered> MultiSam = std::make_shared<MultiJittered>(4);
    world->sampler = MultiSam;
    world->max_depth = 2;

 /****Set A Tracer****/
    //world->tracer_ptr = std::make_shared<RayCast>(world);
    world->tracer_ptr = std::make_shared<AreaLighting>(world);
    //world->tracer_ptr = std::make_shared<Whitted>(world);
    //world->tracer_ptr = std::make_shared<PathTrace>(world);
    ////world->tracer_ptr = std::make_shared<GlobalTrace>(world);
    

 /****Materials****/
    // Floor Matt
    std::shared_ptr<Matte>FloorMat = std::make_shared<Matte>(0.20f, 0.25f, Colour{ 0.25f,0.25f,0.25f });
    // Wall Matt
    std::shared_ptr<Matte>WallMat = std::make_shared<Matte>(0.02f, 0.02f, Colour{ 0.02f,0.02f,0.02f });
    // Box Matt
    std::shared_ptr<Matte>BoxMat = std::make_shared<Matte>(0.20f, 0.08f, Yellow);

    // Floor specular material
    std::shared_ptr<Phong>FloorSpc = std::make_shared<Phong>();
    FloorSpc->set_c({ 0.05,0.05,0.05 });//colour
    FloorSpc->set_ka(0.50f);//Set ambient
    FloorSpc->set_kd(0.10f);//Set diffuse
    FloorSpc->set_ks(0.1f); //Set Sepc
    FloorSpc->set_exp(1.0f);//Set Sepc

    // Wall specular materials
    std::shared_ptr<Phong>WallSpc = std::make_shared<Phong>();
    WallSpc->set_c({ 0.2,0.2,0.4 });//colour
    WallSpc->set_ka(0.50f);//Set ambient
    WallSpc->set_kd(0.10f);//Set diffuse
    WallSpc->set_ks(0.0f); //Set Sepc
    WallSpc->set_exp(0.0f);//Set Sepc

    // Red specular material
    std::shared_ptr<Phong>RSpc = std::make_shared<Phong>();
    RSpc->set_c(Red);//colour
    RSpc->set_ka(0.25f);//Set ambient
    RSpc->set_kd(0.10f);//Set diffuse
    RSpc->set_ks(0.30f); //Set Sepc
    RSpc->set_exp(10.0f);//Set Sepc

    // Green specular material
    std::shared_ptr<Phong>GSpc = std::make_shared<Phong>();
    GSpc->set_c(Green);//colour
    GSpc->set_ka(0.25f);//Set ambient
    GSpc->set_kd(0.10f);//Set diffuse
    GSpc->set_ks(0.30f); //Set Sepc
    GSpc->set_exp(10.0f);//Set Sepc

    // Blue specular material
    std::shared_ptr<Phong>BSpc = std::make_shared<Phong>();
    BSpc->set_c(Blue);//colour
    BSpc->set_ka(0.25f);//Set ambient
    BSpc->set_kd(0.10f);//Set diffuse
    BSpc->set_ks(0.30f); //Set Sepc
    BSpc->set_exp(10.0f);//Set Sepc

    // Yellow specular material
    std::shared_ptr<Phong>YSpc = std::make_shared<Phong>();
    YSpc->set_c(Yellow);//colour
    YSpc->set_ka(0.25f);//Set ambient
    YSpc->set_kd(0.10f);//Set diffuse
    YSpc->set_ks(0.30f); //Set Sepc
    YSpc->set_exp(10.0f);//Set Sepc

    // Reflective material
    std::shared_ptr<Reflective> ReflecW = std::make_shared<Reflective>();
    ReflecW->set_ka(0.0);
    ReflecW->set_kd(0.0);
    ReflecW->set_ks(0.0);
    ReflecW->set_cd({0,0,0});
    ReflecW->set_kr(0.9f);
    ReflecW->set_cr(white);

    // Reflective material
    std::shared_ptr<Reflective> ReflecR = std::make_shared<Reflective>();
    ReflecR->set_ka(0.0);
    ReflecR->set_kd(0.0);
    ReflecR->set_ks(0.0);
    ReflecR->set_cd({ 0,0,0 });
    ReflecR->set_kr(0.9f);
    ReflecR->set_cr(PaleRed);

    // Reflective material
    std::shared_ptr<Reflective> ReflecB = std::make_shared<Reflective>();
    ReflecB->set_ka(0.0);
    ReflecB->set_kd(0.0);
    ReflecB->set_ks(0.0);
    ReflecB->set_cd({ 0,0,0 });
    ReflecB->set_kr(0.9f);
    ReflecB->set_cr(PaleBlue);

    // Reflective material
    std::shared_ptr<Reflective> ReflecG = std::make_shared<Reflective>();
    ReflecG->set_ka(0.0);
    ReflecG->set_kd(0.0);
    ReflecG->set_ks(0.0);
    ReflecG->set_cd({ 0,0,0 });
    ReflecG->set_kr(0.9f);
    ReflecG->set_cr(PaleGreen);

    // PURE Reflective material
    std::shared_ptr<GlossyReflector> RefG = std::make_shared<GlossyReflector>();
    float exp2 = 100000.0;     // roughness, smaller rougher
    RefG->set_samples(1, exp2); // Multijittered(num), maptohemisphere(exp)
    RefG->set_ka(0.0f);
    RefG->set_kd(0.0f);
    RefG->set_ks(0.0f);
    RefG->set_exp(exp2);
    RefG->set_cd(PaleRed);
    RefG->set_kr(10.0f);
    RefG->set_exponent(exp2);
    RefG->set_cr(PaleRed); // red

    // Glossy Reflective material
    std::shared_ptr<GlossyReflector> GlossyR = std::make_shared<GlossyReflector>();
    float exp = 300.0;     // roughness, smaller rougher
    GlossyR->set_samples(25, exp); // Multijittered(num), maptohemisphere(exp)
    GlossyR->set_ka(0.0f);
    GlossyR->set_kd(0.0f); 
    GlossyR->set_ks(0.0f);
    GlossyR->set_exp(exp);
    GlossyR->set_cd(PaleRed);
    GlossyR->set_kr(0.9f);
    GlossyR->set_exponent(exp);
    GlossyR->set_cr(PaleRed); // red


    // Glossy Reflective material
    std::shared_ptr<GlossyReflector> GlossyY = std::make_shared<GlossyReflector>(); 
    GlossyY->set_samples(25, exp);
    GlossyY->set_ka(0.0);
    GlossyY->set_kd(0.0);
    GlossyY->set_ks(0.0);
    GlossyY->set_exp(exp);
    GlossyY->set_cd(1.0f, 1.0f, 0.3f);
    GlossyY->set_kr(0.9f);
    GlossyY->set_exponent(exp);
    GlossyY->set_cr(1.0f, 1.0f, 0.3f); // lemon


    // Glossy Reflective material
    std::shared_ptr<GlossyReflector> GlossyB = std::make_shared<GlossyReflector>();
    GlossyB->set_samples(25, exp);
    GlossyB->set_ka(0.0);
    GlossyB->set_kd(0.0);
    GlossyB->set_ks(0.0);
    GlossyB->set_exp(exp);
    GlossyB->set_cd(PaleBlue);
    GlossyB->set_kr(0.9f);
    GlossyB->set_exponent(exp);
    GlossyB->set_cr(PaleBlue); 

    // Glossy Reflective material
    std::shared_ptr<GlossyReflector> GlossyG = std::make_shared<GlossyReflector>();
    GlossyG->set_samples(25, exp);
    GlossyG->set_ka(0.0);
    GlossyG->set_kd(0.0);
    GlossyG->set_ks(0.0);
    GlossyG->set_exp(exp);
    GlossyG->set_cd(0.3f,1,0.3f);
    GlossyG->set_kr(0.9f);
    GlossyG->set_exponent(exp);
    GlossyG->set_cr(0.3f, 1, 0.3f);

    // Realistic Trasparant Dielectric
    std::shared_ptr<Dielectric> WGlass = std::make_shared <Dielectric>();
    WGlass->set_ks(0.1f);
    WGlass->set_exp(2000.0f);
    WGlass->set_eta_in(1.5f); 			// diamond
    WGlass->set_eta_out(1.0f);			// glass
    WGlass->set_cf_in(0.95f, 0.95f, 1);
    WGlass->set_cf_out(1, 1, 1);

    // Realistic Trasparant Dielectric
    std::shared_ptr<Dielectric> DGlass = std::make_shared <Dielectric>();
    DGlass->set_ks(0.1f);
    DGlass->set_exp(2000.0f);
    DGlass->set_eta_in(2.42f); 			// diamond
    DGlass->set_eta_out(1.5f);			// glass
    DGlass->set_cf_in(1, 1, 0.8f);
    DGlass->set_cf_out(0.95f, 0.95f, 1);

    // transparent sphere
	std::shared_ptr<Transparent> glass_ptr = std::make_shared<Transparent>();
	glass_ptr->set_ks(0.2f);
	glass_ptr->set_exp(2000.0f);			
    glass_ptr->set_ior(1.5f);		
	glass_ptr->set_kr(0.1f);
	glass_ptr->set_kt(0.9f);
	glass_ptr->set_ca(0.0,0.0,0.0);
	glass_ptr->set_cd(0.0,0.0,0.0);
	glass_ptr->set_cs(0.0,0.0,0.0);


    //// RectangularMap
    //Image* image_ptr = new Image;
    //image_ptr->read_ppm_file("checkerboard.ppm");
    //RectangularMap* map_ptr = new RectangularMap;
    //ImageTexture* texture_ptr = new ImageTexture(image_ptr); 
    //texture_ptr->set_mapping(map_ptr);
  
    //std::shared_ptr<SV_Matte> Floor_sv_matte = std::make_shared<SV_Matte>();		
    //Floor_sv_matte->set_ka(0.40f);
    //Floor_sv_matte->set_kd(0.95f);
    //Floor_sv_matte->set_cd(texture_ptr);


    // cylinder mapping
    //Image* image_ptr = new Image;
    //	image_ptr->read_ppm_file("../../TextureFiles/ppm/CountryScene.ppm");
    //	CylindricalMap* map_ptr = new CylindricalMap;   
    //	ImageTexture* texture_ptr = new ImageTexture(image_ptr); 
    //	texture_ptr->set_mapping(map_ptr);
    //	
    //	SV_Matte* sv_matte_ptr = new SV_Matte;		
    //	sv_matte_ptr->set_ka(0.40);
    //	sv_matte_ptr->set_kd(0.95);
    //	sv_matte_ptr->set_cd(texture_ptr);
  	
  
    //// Earth- spherical mapping
    Image* image_ptr3 = new Image;
    image_ptr3->read_ppm_file("../EarthWithClouds.ppm");
    //image_ptr->read_ppm_file("../../TextureFiles/ppm/EarthHighRes.ppm");
    SphericalMap* map_ptr3 = new SphericalMap;
    ImageTexture* texture_ptr3 = new ImageTexture; 
    texture_ptr3->set_image(image_ptr3); 
    texture_ptr3->set_mapping(map_ptr3);

    TInstance* earth_Tins = new TInstance(texture_ptr3);
    earth_Tins->translate(0, 0);
    earth_Tins->scale(150,150,150);

     //textured material:
    std::shared_ptr<SV_Matte> S_matte_ptr = std::make_shared<SV_Matte>();
	S_matte_ptr->set_ka(0.8f);
	S_matte_ptr->set_kd(0.55f);
	S_matte_ptr->set_cd(earth_Tins);

    Sphere* sphere_ptr = new Sphere({ 2,0,-3 }, 1);
    sphere_ptr->setMaterial(S_matte_ptr);
    std::shared_ptr<Instance> earth_ptr = std::make_shared<Instance>(sphere_ptr);
    //earth_ptr->translate({ 0, 50, 0 });
    earth_ptr->scale(150);
    earth_ptr->compute_bounding_box();


    //  Procedural texture 
    Checker3D* checker_ptr = new Checker3D;
    checker_ptr->set_size(0.3f);
    checker_ptr->set_color1(white);
    checker_ptr->set_color2(black);

    TInstance* checker_ptr2 = new TInstance(checker_ptr);
    checker_ptr2->scale(90,90,90);

    std::shared_ptr<SV_Matte> sv_matte_ptr = std::make_shared<SV_Matte>();
    sv_matte_ptr->set_ka(0.8f);
    sv_matte_ptr->set_kd(0.4f);
    sv_matte_ptr->set_cd(checker_ptr2);

    Box* box_ptr1 = new Box({ -4.5,0,0 }, { -2.5, 2, 2 });
    box_ptr1->setMaterial(sv_matte_ptr);

    std::shared_ptr<Instance> box_ptr = std::make_shared<Instance>(box_ptr1);
   // box_ptr->translate(-50, 10, 40);
    box_ptr->scale(90,90,90);
    box_ptr->compute_bounding_box();
    //box_ptr->rotate_z(45);

    //  Procedural texture 
    std::shared_ptr<Checker3D> checker2 = std::make_shared<Checker3D>();
    checker2->set_size(0.3f);
    checker2->set_color1(Red);
    checker2->set_color2(white);

    TInstance* checker_ptr3 = new TInstance(checker2.get());
    checker_ptr3->scale(50, 50, 50);

    std::shared_ptr<SV_Matte> sv_matte_ptr2 = std::make_shared<SV_Matte>();
    sv_matte_ptr2->set_ka(0.9f);
    sv_matte_ptr2->set_kd(0.1f);
    sv_matte_ptr2->set_cd(checker_ptr3);


    //////Noise-Based Texture
    //CubicNoise* noise_ptr = new CubicNoise;
    //noise_ptr->set_num_octaves(6);
    //noise_ptr->set_gain(0.5f);
    //noise_ptr->set_lacunarity(8);

    //// the texture:
    //FBmTextureWrapped* fBm_texture_ptr = new FBmTextureWrapped(noise_ptr);
    //fBm_texture_ptr->set_color({ 0.7f, 1.0f, 0.5f });   // light green
    //fBm_texture_ptr->set_min_value(-0.1f);
    //fBm_texture_ptr->set_max_value(1.1f);

    //// the material:
    //std::shared_ptr<SV_Matte> sv_noise_ptr = std::make_shared<SV_Matte>();
    //sv_noise_ptr->set_ka(0.25f);
    //sv_noise_ptr->set_kd(0.85f);
    //sv_noise_ptr->set_cd(fBm_texture_ptr);

    //// the object
    //std::shared_ptr<Sphere>NoiseSphere = std::make_shared<Sphere>(atlas::math::Point(0.0), 300);
    //NoiseSphere->setMaterial(sv_noise_ptr);


/****Grid****/
    bool use_grid = true;
    Grid* grid_ptr = NULL;
    if (use_grid) {
        grid_ptr = new Grid;
        srand(15);
    }

 /***Rendering a Mesh here***/
    std::shared_ptr<Grid>mesh_obj_ptr = std::make_shared<Grid>(new Mesh);
    //mesh_grid_ptr->reverse_mesh_normals();
    mesh_obj_ptr->tessellate_flat_sphere(10, 6);
    mesh_obj_ptr->setup_cells();
    //mesh_obj_ptr->setMaterial(glass_ptr);
    std::shared_ptr<Instance> mesh_instance = std::make_shared<Instance>(mesh_obj_ptr.get());
    //mesh_instance->translate(0,0.1f,0);
    mesh_instance->scale(150, 150, 150);
    mesh_instance->compute_bounding_box();
    mesh_instance->setMaterial(glass_ptr);
    world->add_object(mesh_instance);
 /***Rendered a Mesh***/

///***Set some Objects***/
    std::shared_ptr<Plane> Wall = std::make_shared<Plane>(atlas::math::Point{ 0, 0, -2000 }, atlas::math::Vector{ 0,0,1 });
    Wall->setMaterial(WallMat);
    std::shared_ptr<Plane> Floor = std::make_shared<Plane>(atlas::math::Point{ 0, -150, 0 }, atlas::math::Vector{ 0,1,0 });
    Floor->setMaterial(FloorMat);
    std::shared_ptr<Sphere> Ball1 = std::make_shared<Sphere>(atlas::math::Point{ 70, -50, 380 }, 100.0f);
    //std::shared_ptr<Sphere> Ball1 = std::make_shared<Sphere>(atlas::math::Point{ 0, 90, 0 }, 90.0f);
    Ball1->setMaterial(ReflecR);
    std::shared_ptr<Sphere> Ball2 = std::make_shared<Sphere>(atlas::math::Point{ -230, -75, 250 }, 85.0f);
    Ball2->setMaterial(GlossyG);
    std::shared_ptr<Sphere> Ball3 = std::make_shared<Sphere>(atlas::math::Point{ 310, -75, 280 }, 85.0f);
    Ball3->setMaterial(BSpc);
    std::shared_ptr<Sphere> Ball4 = std::make_shared<Sphere>(atlas::math::Point{ -380, -50, 440 }, 100.0f);
    Ball4->setMaterial(GlossyB);
    std::shared_ptr<Sphere> Ball5 = std::make_shared<Sphere>(atlas::math::Point{ 235, -105, 530 }, 45.0f);
    Ball5->setMaterial(GSpc);
    std::shared_ptr<Sphere> Ball6 = std::make_shared<Sphere>(atlas::math::Point{ -145, -105, 560 }, 45.0f);
    Ball6->setMaterial(RSpc);
    std::shared_ptr<Sphere> Ball7 = std::make_shared<Sphere>(atlas::math::Point{ 30, -100, 670 }, 50.0f);
    Ball7->setMaterial(sv_matte_ptr2);
    std::shared_ptr<Sphere> Ball8 = std::make_shared<Sphere>(atlas::math::Point{ 380, -90, 450 }, 60.0f);
    Ball8->setMaterial(WGlass);
    std::shared_ptr<Sphere> Ball9 = std::make_shared<Sphere>(atlas::math::Point{ -235, -105, 650 }, 45.0f);
    Ball9->setMaterial(ReflecW);
    std::shared_ptr<Sphere> Ball10 = std::make_shared<Sphere>(atlas::math::Point{ 380, -90, 450 }, 30.0f);
    Ball10->setMaterial(YSpc);
    std::shared_ptr<Sphere> Ball11 = std::make_shared<Sphere>(atlas::math::Point{ 155, -105, 650 }, 45.0f);
    Ball11->setMaterial(glass_ptr);
    std::shared_ptr<Cylinder> Cyli1 = std::make_shared<Cylinder>(-150.0f, 100.0f, 90.0f);
    Cyli1->setMaterial(BoxMat);
    std::shared_ptr<Box> Box1 = std::make_shared<Box>(atlas::math::Point{ 330, -150, -100 }, atlas::math::Point{ 600, 60 ,-50 });
    Box1->setMaterial(glass_ptr);
    std::shared_ptr<Box> Box2 = std::make_shared<Box>(atlas::math::Point{ -650, -150, -400 }, atlas::math::Point{-400, 0,-250 });
    Box2->setMaterial(glass_ptr);
    world->add_object(Wall);
    world->add_object(Floor);


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*****Render without grid****/

    //world->add_object(Ball1);
    //world->add_object(Ball2);
    //world->add_object(Ball3);
    //world->add_object(Ball4);
    //world->add_object(Ball5);
    //world->add_object(Ball6);
    //world->add_object(Ball7);
    //world->add_object(Ball8);
    //world->add_object(Ball9);
    //world->add_object(Ball10);
    //world->add_object(Ball11);
    ////world->add_object(Cyli1);
    //world->add_object(Box1);
    //world->add_object(Box2);
    //world->add_object(earth_ptr);
    //world->add_object(box_ptr);


/*****Render with grid***********/
    std::shared_ptr<Grid>mesh_grid_ptr = std::make_shared<Grid>(new Mesh);
    mesh_grid_ptr->add_object(Ball1.get());  
    mesh_grid_ptr->add_object(Ball2.get());
    mesh_grid_ptr->add_object(Ball3.get());
    mesh_grid_ptr->add_object(Ball4.get()); 
    mesh_grid_ptr->add_object(Ball5.get());
    mesh_grid_ptr->add_object(Ball6.get());
    mesh_grid_ptr->add_object(Ball7.get());
    mesh_grid_ptr->add_object(Ball8.get());
    mesh_grid_ptr->add_object(Ball9.get());
    mesh_grid_ptr->add_object(Ball10.get());
    mesh_grid_ptr->add_object(Ball11.get());
    //mesh_grid_ptr->add_object(Cyli1.get());
    mesh_grid_ptr->add_object(Box1.get());
    mesh_grid_ptr->add_object(Box2.get());
    mesh_grid_ptr->add_object(earth_ptr.get());
    mesh_grid_ptr->add_object(box_ptr.get());
    mesh_grid_ptr->setup_cells();
    world->add_object(mesh_grid_ptr);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   /* std::shared_ptr<Matte>Matt = std::make_shared<Matte>(0.20f, 0.20f, Colour{ 0.5,0.5,0.5 });
    
    world->scene.push_back(
        std::make_shared<Plane>(atlas::math::Point{ 0, 0, -1000 }, atlas::math::Vector{ 0,0,1 }));
    world->scene[1]->setMaterial(
        std::make_shared<Matte>(0.20f, 0.20f, Colour{ 0.2,0.2,0.5 }));
    



    /*world->scene.push_back(
        std::make_shared<Cone>(atlas::math::Point(-70, 0, 120), 150.0f, 40.0f));
    world->scene[4]->setMaterial(
        std::make_shared<Matte>(0.25f, 0.10f, Colour{ 1, 1, 0 }));*/
    //world->scene[4]->setColour({ 0, 0, 1 });
 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*Ambient Light*/
    //world->ambient = std::make_shared<Ambient>();
    ////set lights parameter
    //world->ambient->setColour({ 1, 1, 1 });
    //world->ambient->scaleRadiance(2.0f);


/*Ambient Occlusion*/
    std::shared_ptr<AmbientOccluder> AmO = std::make_shared<AmbientOccluder>();
    AmO->scaleRadiance(2.0f);
    AmO->set_min_amount(0.2f);
    AmO->set_sampler(new MultiJittered(4,20));
    world->set_ambient_light(AmO);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


/*Directional Light*/
    //std::shared_ptr <Directional> dirL = std::make_shared<Directional>();
    //dirL->setDirection({ 0.25f, 0.4f, -1 });
    //dirL->scaleRadiance(1.5f);
    //world->add_light(dirL);
    
/*Point Light*/
    std::shared_ptr pointL = std::make_shared<PointLight>();
    pointL->setLocation({-300.0f, 350.0f, 624.0f });
    pointL->scaleRadiance(6.5f);
    pointL->setColor({ 1,1,1 });
    world->add_light(pointL);   


/*Point Light*/
    ////std::shared_ptr pointL = std::make_shared<PointLight>();
    ////pointL->setLocation({2.0f, 2.0f, 2.0f });
    ////pointL->scaleRadiance(2.5f);
    ////pointL->setColor({ 1,1,1 });
    ////world->add_light(pointL);   


/*Fake Light*/
    //std::shared_ptr fakeL = std::make_shared<FakeSphericalLight>();
    //fakeL->setLocation({ -300.0f, 600.0f, 124.0f });
    //fakeL->scaleRadiance(5.5f);
    //fakeL->setColor({ 1,1,1 });
    //fakeL->setRadius(20.0f);
    //world->add_light(fakeL);
    

/*Box and Disk Objects*/
   /* world->scene.push_back(
        std::make_shared<Box>(atlas::math::Point{-250, 0, 200 }, atlas::math::Point{ -200, 50 ,250 }));*/
    /*world->scene.push_back(
        std::make_shared<Disk>(atlas::math::Point{ 50,400, 60 }, 50.0f, atlas::math::Vector{ 0, -1 ,0 }));*/
    
///*Area Light Object: Rectangle AreaLight*/
    std::shared_ptr<Emissive> Emm{ std::make_shared<Emissive>() };
    Emm->scale_radiance(70.0f);
    Emm->set_ce({ 1,1,1 });
    std::shared_ptr<Rectangle> Rec{ std::make_shared<Rectangle>(atlas::math::Point{ -150,440, 440 }, atlas::math::Vector{ 0, 0 ,-300 }, atlas::math::Vector{ 300, 0 ,0 }) };
    Rec->setMaterial(Emm);
    Rec->set_sampler(MultiSam.get());
    Rec->set_shadows(false);
    world->add_object(Rec);

    std::shared_ptr<AreaLight> areaL{ std::make_shared<AreaLight>() };
    areaL->set_shadows(true);
    areaL->set_object(Rec);
    world->add_light(areaL);


///*Area Light Object: Spherical AreaLight*/
    //std::shared_ptr<Emissive> Emm2{ std::make_shared<Emissive>() };
    //Emm2->scale_radiance(5.0f);
    //Emm2->set_ce({ 1,1,1 });
    //std::shared_ptr<Sphere> Sphe{ std::make_shared<Sphere>(atlas::math::Point{ -300, 400, 324 }, 30.0f) };
    //Sphe->setMaterial(Emm2);
    //Sphe->set_sampler(new MultiJittered(16, 80));
    //Sphe->set_shadows(false);
    //world->add_object(Sphe);

    //std::shared_ptr<AreaLight> areaL2{ std::make_shared<AreaLight>() };
    //areaL2->set_shadows(true);
    //areaL2->set_object(Sphe);
    //world->add_light(areaL2);


///*Area Light Object: Rectangle AreaLight*/
    std::shared_ptr<Emissive> Emm2{ std::make_shared<Emissive>() };
    Emm2->scale_radiance(70.0f);
    Emm2->set_ce({ 1,1,1 });
    std::shared_ptr<Rectangle> Rec2{ std::make_shared<Rectangle>(atlas::math::Point{ -150,440, 1040 }, atlas::math::Vector{ 0, 0 ,-300 }, atlas::math::Vector{ 300, 0 ,0 }) };
    Rec2->setMaterial(Emm2);
    Rec2->set_sampler(MultiSam.get());
    Rec2->set_shadows(false);
    world->add_object(Rec2);

    std::shared_ptr<AreaLight> areaL2{ std::make_shared<AreaLight>() };
    areaL2->set_shadows(true);
    areaL2->set_object(Rec2);
    world->add_light(areaL2);

/*Instance*/
    //Sphere* sp = new Sphere(atlas::math::Point{ 125, 90, 150 }, 90.0f);
    //sp->setMaterial(RSpc);
    //std::shared_ptr<Instance> instance = std::make_shared<Instance>(sp);
    ////instance->scale(75, 75, 75);
    //instance->translate(.5, -5.25, 0);
    //instance->compute_bounding_box();
    //world->add_object(instance);



    // set up camera
    Pinhole camera{};

    // change camera position here
    camera.setEye({ 0.0f, 350.0f, 1500.0f });
    camera.setLookAt({ 0.0f, 70.0f, 0.0f });
    camera.setDistance(1300.0);
    //camera.setZoom(4.0f/7.0f);

    camera.compute_uvw();

    //
    //camera.setEye({ 0, 20, 400});
    //camera.setLookAt({ 0.0f, 0.0f, 0.0f });
    ////camera.setDistance(210.0);   
    //camera.setDistance(21000.0);
    ////camera.setZoom(4.0f/7.0f);
    //camera.compute_uvw();
    
    

    auto start = std::chrono::steady_clock::now();
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
///***Single thread***/
    camera.renderScene(world);
    

///***Intermediate parallelization***/
    //multi_threads(world, camera);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Rendering time: " << elapsed_seconds.count() << "s\n";

    saveToBMP("AS4.bmp", world->width, world->height, world->image);

    return 0;
}




void multi_threads(std::shared_ptr<World>& world, const Pinhole pinhole) {
    // divide the image into n×n slabs, where n is larger than the maximum number of threads 
    if (world->width != world->height) {
        printf("Please make width and height equal.\n");
    }

    int size = Slabs_num;
    int slab_length = (int)(world->width / sqrt(size));
    printf("\ndivide the image into %d slabs\n", size);
    world->slabs.reserve(Slabs_num*sizeof(Slab));
    for (int v = 0; v < sqrt(size); v++) {
        for (int h = 0; h < sqrt(size); h++) {
            Point2D start (h * slab_length, v * slab_length);
            Point2D end(start.x + slab_length, start.y + slab_length);
            world->slabs.push_back(std::make_shared<Slab>(start, end));
        }
    }
    /*for (int i = 0; i < world->slabs.size(); i++)
    {
        printf("slab (%d, %d)(%d, %d)\n", world->slabs[i]->start.x, world->slabs[i]->start.y, world->slabs[i]->end.x, world->slabs[i]->end.y);
    }*/
    int pixel_num = (int)(world->width * world->height);
    //Colour Black = { 0,0,0 };
    //std::fill(world->image.begin(), world->image.begin() + 160000, Black);
    //world->image = new std::vector<Colour>(size);
    //world->image.reserve((int)(world->width * world->height)*sizeof(Colour));
    world->image.resize(pixel_num);
    //printf("Number of Pixels: %d\n", (int)world->image.size());
    remain_slabs = size;
    int n_Threads = std::thread::hardware_concurrency();   // max num of threads the sys supports
    printf("Max: %d threads start.\n", n_Threads-1);
    std::thread* Render_threads = new std::thread[n_Threads];
    

    std::vector<std::shared_ptr<ShadeRec>> trace_data(n_Threads);
    for (int i = 0; i < n_Threads; i++) {
        trace_data[i] = std::make_shared<ShadeRec>(*world);
    }
    //trace_data.world = *world;
    
    // launch threads
    for (int i = 0; i < n_Threads - 1; i++) {
        Render_threads[i] = std::thread(render_all, std::ref(world), pinhole, std::ref(trace_data[i]));
    }
    for (int i = 0; i < n_Threads - 1; i++) {
        Render_threads[i].join();
    }

    printf("All threads done!\n");
}

void render_all(std::shared_ptr<World>& world, const Pinhole pinhole, std::shared_ptr<ShadeRec>& trace_data) {
    while (1) {
        if (remain_slabs==-1) {
            printf("a thread exit!\n");
            return;
        }
        mutex.lock();   //sem_wait(&mutex);
        if (remain_slabs == 0) {
            remain_slabs--;
            mutex.unlock();
            break;
        }
        //printf("mutexlock\n");
        //slab_process();
        int i = 0;
        for (i = 0; i < world->slabs.size(); i++)
        {
            if(world->slabs[i]->status==0){
                world->slabs[i]->status = 1;
                break;
            }
        }
        //printf("get %d slab to start.\n",i);
        remain_slabs--;
        if(remain_slabs%10 ==0)
            printf("remain slabs:%d\n", remain_slabs);
        //sem_post(&mutex);
        mutex.unlock();
        render_slab(world->slabs[i], world, pinhole, trace_data);
    }
    //render_edge(world, pinhole);
    printf("a thread exit!\n");
    return;
}


void render_slab(std::shared_ptr<Slab> slab, std::shared_ptr<World>& world, const Pinhole pinhole, std::shared_ptr<ShadeRec>& trace_data) {
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;
    Point samplePoint{}, pixelPoint{};
    //Ray<atlas::math::Vector> ray{};
    //int depth = 0; //depth start from 0;
    float s = world->s / pinhole.mZoom;
    trace_data->ray.o = pinhole.mEye;
    //int length = slab->end.x - slab->start.x;

    for (int r = slab->start.y; r < slab->end.y; r++) {	// up 
        for (int c = slab->start.x; c < slab->end.x; c++) {
            Colour L { 0,0,0 };
            for (int j = 0; j < trace_data->world.sampler->getNumSamples(); j++) {
  
                samplePoint = trace_data->world.sampler->sampleUnitSquare();
                pixelPoint.x = (float)(s * (c - 0.5 * world->width + samplePoint.x));
                pixelPoint.y = -(float)(s * (r - 0.5 * world->height + samplePoint.y));
                trace_data->ray.d = pinhole.rayDirection(pixelPoint);
          
                L += world->tracer_ptr->trace_ray(trace_data->ray, 0);
      
            }
            float avg{ 1.0f / trace_data->world.sampler->getNumSamples() };
            L *= avg;
            colour_handling(L);
            //out-of-gamut control
            int index = (int)((r * world->width) + c);
            world->image[index] = L;
        }
    }
}


void render_edge(std::shared_ptr<World>& world, const Pinhole pinhole) {
    printf("start render_edge\n");
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;
    Point samplePoint{}, pixelPoint{};
    Ray<atlas::math::Vector> ray{};
    int depth = 0; //depth start from 0;

    float s = world->s / pinhole.mZoom;
    ray.o = pinhole.mEye;

    for (int r =0 ; r < world->height-1; r++) {	// up 
        Colour L{ 0,0,0 };
        for (int j = 0; j < world->sampler->getNumSamples(); j++) {
            //printf("get sample\n");
            samplePoint = world->sampler->sampleUnitSquare();
            //printf("(%f,%f):\n", pixelPoint.x, pixelPoint.y);
            pixelPoint.x = (float)(s * (-1 + 0.5 * world->width + samplePoint.x));
            pixelPoint.y = -(float)(s * (r - 0.5 * world->height + samplePoint.y));
            ray.d = pinhole.rayDirection(pixelPoint);
            //printf("A\n");
            L += world->tracer_ptr->trace_ray(ray, depth);
        }
        float avg{ 1.0f / world->sampler->getNumSamples() };
        L *= avg;
        colour_handling(L);
        //out-of-gamut control 
        int verti = (int) (r * world->width + (world->width - 1));
        //printf("c\n");
        world->image.assign(verti, L);
        //printf("(%f,%f): [%d]\n", pixelPoint.x, pixelPoint.y, verti);
        //printf("d\n");
    }
    //printf("Vertical done\n");
    for (int c = 0; c < world->width; c++) {
        Colour H{ 0,0,0 };
        for (int j = 0; j < world->sampler->getNumSamples(); j++) {
            samplePoint = world->sampler->sampleUnitSquare();
            pixelPoint.x = (float)(s * (c - 0.5 * world->width + samplePoint.x));
            pixelPoint.y = -(float)(s * (0.5 * world->height + samplePoint.y));
            ray.d = pinhole.rayDirection(pixelPoint);
            H += world->tracer_ptr->trace_ray(ray, depth);
        }
        float avg{ 1.0f / world->sampler->getNumSamples() };
        H *= avg;
        colour_handling(H);
        //out-of-gamut control
        world->image.assign(((int)((world->height * world->width- world->width) + c)), H);
    }
    //printf("Horizontal done\n");
}

void slab_process() {
    int size = Slabs_num;
    printf("Process: %f%%\n", ((size-remain_slabs)*1.0/size*1.0)*100);
}

void colour_handling(Colour& c)
{
    if (c.r <= 1.0f && c.g <= 1.0f && c.b <= 1.0f) return;

    float scaler{ c.r };
    if (c.g > scaler) scaler = c.g;
    if (c.b > scaler) scaler = c.b;

    scaler = 1.0f / scaler;
    c.r *= scaler;
    c.g *= scaler;
    c.b *= scaler;
    return;
}

/**
 * Saves a BMP image file based on the given array of pixels. All pixel values
 * have to be in the range [0, 1].
 *
 * @param filename The name of the file to save to.
 * @param width The width of the image.
 * @param height The height of the image.
 * @param image The array of pixels representing the image.
 */
void saveToBMP(std::string const& filename,
               std::size_t width,
               std::size_t height,
               std::vector<Colour> const& image)
{
    std::vector<unsigned char> data(image.size() * 3);

    for (std::size_t i{0}, k{0}; i < image.size(); ++i, k += 3)
    {
        Colour pixel = image[i];
        data[k + 0]  = static_cast<unsigned char>(pixel.r * 255);
        data[k + 1]  = static_cast<unsigned char>(pixel.g * 255);
        data[k + 2]  = static_cast<unsigned char>(pixel.b * 255);
    }

    stbi_write_bmp(filename.c_str(),
                   static_cast<int>(width),
                   static_cast<int>(height),
                   3,
                   data.data());
}



//ShadeRec trace_data(*world);
                //trace_data.world = *world;

                //double t;
                //Vector normal = { 0,0,0 };
                //Point local_hit_point = { 0,0,0 };
                //float tmin = (float)1.0E10;
                //int	num_objects = (int)world->scene.size();
                //for (int o = 0; o < num_objects; o++) {

                //    if (world->scene[o]->hit(trace_data->ray, t, *trace_data) && (t < tmin)) {
                //        trace_data->hit_an_object = true;
                //        tmin = (float)t;
                //        trace_data->material = NULL;
                //        trace_data->material = world->scene[o]->getMaterial();
                //        if (trace_data->material == NULL) {
                //            printf("no get!\n");
                //            exit(0);
                //        }

                //        trace_data->local_hit_point = trace_data->ray.o + (float)t * trace_data->ray.d;
                //        normal = trace_data->normal;
                //        local_hit_point = trace_data->local_hit_point;
                //    }
                //}
                ////mutex.unlock();
                //if (trace_data->hit_an_object) {
                //    trace_data->t = tmin;
                //    trace_data->normal = normal;
                //    trace_data->local_hit_point = local_hit_point;
                //}
                //if (trace_data->hit_an_object) {
                //    L += trace_data->material->area_light_shade(*trace_data);
                //}
                //trace_data->t = std::numeric_limits<float>::max();
                //trace_data->hit_an_object = false;
                //for (auto obj : trace_data->world.scene)
                //{
                //    //trace_data->hit_an_object |= obj->hit(trace_data->ray, trace_data->t, *trace_data);
                //    if (obj->hit(trace_data->ray, trace_data->t, *trace_data)) {
                //        trace_data->hit_an_object = true;
                //        trace_data->material = obj->getMaterial();
                //        trace_data->local_hit_point = trace_data->ray.o + (float)trace_data->t * trace_data->ray.d;
                //    }
                //}
                //mutex.unlock();
                ///*if (){}
                //    trace_data->material = NULL;
                //    trace_data->hit_an_object = false;*/
                //if (trace_data->hit_an_object) {
                //    L += trace_data->material->area_light_shade(*trace_data);
                //    //L += trace_data.material->path_shade(trace_data);
                //}

                //trace_data = std::make_shared<ShadeRec>(*world);
                //Colour temp = { 0,0,0 };
                //ShadeRec sr(world->hit_objects(trace_data->ray)); // sr is copy constructed
                //mutex.unlock();
                //sr.depth = 0;
                //printf("sr depth is %d\n", sr.depth);
                //if (sr.hit_an_object) {


                //}


                ////////////////////////////////
                //L += world->tracer_ptr->trace_ray(ray, depth);   
                //trace_data->temp_tmin = std::numeric_limits<float>::max();
                //trace_data->hit_an_object = false;
                //for (int o = 0; o < (int)world->scene.size(); o++) {
                //    if (trace_data->world.scene[o]->hit(trace_data->ray, trace_data->t, *trace_data) && (trace_data->t < trace_data->temp_tmin)) {
                //        trace_data->hit_an_object = true;
                //        trace_data->temp_tmin = trace_data->t;
                //        trace_data->material = trace_data->world.scene[o]->getMaterial();
                //        trace_data->local_hit_point = trace_data->ray.o + (float)trace_data->t * trace_data->ray.d;
                //        trace_data->temp_normal = trace_data->normal;
                //        trace_data->temp_hit_point = trace_data->local_hit_point;
                //    }
                //}
                //if (trace_data->hit_an_object) {
                //    trace_data->t = trace_data->temp_tmin;
                //    trace_data->normal = trace_data->temp_normal;
                //    trace_data->local_hit_point = trace_data->temp_hit_point;
                //    //trace_data->material = trace_data->temp_material;
                //    //L += trace_data->material->area_light_shade(*trace_data);
                //}
                //mutex.unlock();
                //if (trace_data->hit_an_object) 
                //    L += trace_data->material->area_light_shade(*trace_data);  