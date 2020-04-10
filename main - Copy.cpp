#include "assignment.hpp"
#include <chrono>

// ******* Function Member Implementation *******
// ***** Instance *****
Matrix Instance::forward_matrix;	
void
Instance::compute_bounding_box(void) {
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
    Colour black { 0,0,0 };
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
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;
    double t;
    Vector normal = { 0,0,0 };
    Point local_hit_point = { 0,0,0 };
    float tmin = (float)1.0E10;
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
Camera::Camera() :
    mEye{ 0.0f, 0.0f, 500.0f },
    mLookAt{ 0.0f },
    mUp{ 0.0f, 1.0f, 0.0f },
    mU{ 1.0f, 0.0f, 0.0f },
    mV{ 0.0f, 1.0f, 0.0f },
    mW{ 0.0f, 0.0f, 1.0f }
{}

void Camera::setEye(atlas::math::Point const& eye)
{
    mEye = eye;
}

void Camera::setLookAt(atlas::math::Point const& lookAt)
{
    mLookAt = lookAt;
}

void Camera::setUpVector(atlas::math::Vector const& up)
{
    mUp = up;
}

void Camera::computeUVW()
{
    mW = glm::normalize(mEye - mLookAt);
    mU = glm::normalize(glm::cross(mUp, mW));
    mV = glm::cross(mW, mU);
    if (atlas::core::areEqual(mEye.x, mLookAt.x) && atlas::core::areEqual(mEye.z, mLookAt.z) &&
        mEye.y > mLookAt.y)
    {
        mU = { 0.0f, 0.0f, 1.0f };
        mV = { 1.0f, 0.0f, 0.0f };
        mW = { 0.0f, 1.0f, 0.0f };
    }
    if (atlas::core::areEqual(mEye.x, mLookAt.x) && atlas::core::areEqual(mEye.z, mLookAt.z) &&
        mEye.y < mLookAt.y)
    {
        mU = { 1.0f, 0.0f, 0.0f };
        mV = { 0.0f, 0.0f, 1.0f };
        mW = { 0.0f, -1.0f, 0.0f };
    }
}

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
    const auto dir = p.x * mU + p.y * mV - mDistance * mW;
    return glm::normalize(dir);
}


void Pinhole::renderScene(std::shared_ptr<World> world) const
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
            Colour black = { 0,0,0 };
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



// ***** Light function members *****
void Light::scaleRadiance(float b)
{
    mRadiance = b;
}

void Light::setColour(Colour const& c)
{
    mColour = c;
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


int main()
{
    // Your code here.
    using atlas::math::Point;
    using atlas::math::Ray;
    using atlas::math::Vector;


    std::shared_ptr<World> world{ std::make_shared<World>() };
    // provide world data
    world->width = 1000;                 //$$$$$$$$$$$$$$$$$$$$$$$fix this
    world->height = 1000;                //$$$$$$$$$$$$$$$$$$$$$$$fix this
    world->background = { 0, 0, 0 };
    //world->sampler = std::make_shared<MultiJittered>(49, 83);     //$$$$$$$$$$$$$$$$$$$$$$$fix this
    //world->sampler = std::make_shared<MultiJittered>(16);         //$$$$$$$$$$$$$$$$$$$$$$$fix this
    //world->sampler = std::make_shared<Regular>(1);        //$$$$$$$$$$$$$$$$$$$$$$$fix this
    std::shared_ptr<MultiJittered> MultiSam = std::make_shared<MultiJittered>(36);
    world->sampler = MultiSam;
    world->max_depth = 1;


    //world->tracer_ptr = std::make_shared<RayCast>(world);
    world->tracer_ptr = std::make_shared<AreaLighting>(world);
    //world->tracer_ptr = std::make_shared<Whitted>(world);



 /****Materials****/
    // Floor Matt
    std::shared_ptr<Matte>FloorMat = std::make_shared<Matte>(0.20f, 0.25f, Colour{ 0.3,0.3,0.3 });
    // Wall Matt
    std::shared_ptr<Matte>WallMat = std::make_shared<Matte>(0.20f, 0.20f, Colour{ 0.3,0.3,0.5 });
    // Box Matt
    std::shared_ptr<Matte>BoxMat = std::make_shared<Matte>(0.20f, 0.08f, Yellow);

    // Floor specular material
    std::shared_ptr<Phong>FloorSpc = std::make_shared<Phong>();
    FloorSpc->set_c({ 0.3,0.3,0.3 });//colour
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
    std::shared_ptr<Reflective> ReflecR = std::make_shared<Reflective>();
    ReflecR->set_ka(0.0);
    ReflecR->set_kd(0.0);
    ReflecR->set_ks(0.0);
    ReflecR->set_cd({0,0,0});
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


    // Glossy Reflective material
    std::shared_ptr<GlossyReflector> GlossyR = std::make_shared<GlossyReflector>();
    float exp = 1000.0;     // roughness, smaller rougher
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





/****Grid****/
    bool use_grid = true;
    Grid* grid_ptr = NULL;
    if (use_grid) {
        grid_ptr = new Grid;
        srand(15);
    }

 /***Rendering a Mesh here***/
    //std::shared_ptr<Grid>mesh_grid_ptr = std::make_shared<Grid>(new Mesh);
    ////mesh_grid_ptr->reverse_mesh_normals();
    //mesh_grid_ptr->tessellate_flat_sphere(10, 6);
    //mesh_grid_ptr->setup_cells();
    //mesh_grid_ptr->setMaterial(GSpc);
    //std::shared_ptr<Instance> instance = std::make_shared<Instance>(mesh_grid_ptr.get());
    ////instance->translate({ 0.2,0.2,0.2});
    //instance->scale(150, 150, 150);
    //instance->compute_bounding_box();
    //world->add_object(instance);
    //world->scene[0]->setMaterial(RSpc);
 /***Rendered a Mesh***/

///***Set Objects***/
    std::shared_ptr<Plane> Wall = std::make_shared<Plane>(atlas::math::Point{ 0, 0, -1000 }, atlas::math::Vector{ 0,0,1 });
    Wall->setMaterial(WallMat);
    std::shared_ptr<Plane> Floor = std::make_shared<Plane>(atlas::math::Point{ 0, 0, 0 }, atlas::math::Vector{ 0,1,0 });
    Floor->setMaterial(FloorMat);
    std::shared_ptr<Sphere> Ball1 = std::make_shared<Sphere>(atlas::math::Point{ 125, 90, 150 }, 90.0f);
    Ball1->setMaterial(GlossyR);
    std::shared_ptr<Sphere> Ball2 = std::make_shared<Sphere>(atlas::math::Point{ -230, 85, -130 }, 85.0f);
    Ball2->setMaterial(GSpc);
    std::shared_ptr<Sphere> Ball3 = std::make_shared<Sphere>(atlas::math::Point{ 310, 85, -30 }, 85.0f);
    Ball3->setMaterial(BSpc);
    std::shared_ptr<Sphere> Ball4 = std::make_shared<Sphere>(atlas::math::Point{ -280, 70, 40 }, 70.0f);
    Ball4->setMaterial(GlossyB);
    std::shared_ptr<Sphere> Ball5 = std::make_shared<Sphere>(atlas::math::Point{ 235, 45, 230 }, 45.0f);
    Ball5->setMaterial(GSpc);
    std::shared_ptr<Sphere> Ball6 = std::make_shared<Sphere>(atlas::math::Point{ -145, 45, 260 }, 45.0f);
    Ball6->setMaterial(RSpc);
    std::shared_ptr<Sphere> Ball7 = std::make_shared<Sphere>(atlas::math::Point{ 30, 50, 270 }, 50.0f);
    Ball7->setMaterial(YSpc);
    std::shared_ptr<Cylinder> Cyli1 = std::make_shared<Cylinder>(0.0f, 250.0f, 90.0f);
    Cyli1->setMaterial(GlossyY);
    std::shared_ptr<Box> Box1 = std::make_shared<Box>(atlas::math::Point{ 150, 0, -350 }, atlas::math::Point{ 400, 250 ,-100 });
    Box1->setMaterial(YSpc);
    std::shared_ptr<Box> Box2 = std::make_shared<Box>(atlas::math::Point{ -550, 0, -400 }, atlas::math::Point{-400, 150 ,-250 });
    Box2->setMaterial(YSpc);
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
    //world->add_object(Cyli1);
    //world->add_object(Box1);
    //world->add_object(Box2);
 

/*****Render with grid****/
    std::shared_ptr<Grid>mesh_grid_ptr = std::make_shared<Grid>(new Mesh);
    //mesh_grid_ptr->reverse_mesh_normals();
    //mesh_grid_ptr->tessellate_flat_sphere(10, 6);
    mesh_grid_ptr->add_object(Ball1.get());  
    mesh_grid_ptr->add_object(Ball2.get());
    mesh_grid_ptr->add_object(Ball3.get());
    mesh_grid_ptr->add_object(Ball4.get()); 
    mesh_grid_ptr->add_object(Ball5.get());
    mesh_grid_ptr->add_object(Ball6.get());
    mesh_grid_ptr->add_object(Ball7.get());
    mesh_grid_ptr->add_object(Cyli1.get());
    mesh_grid_ptr->add_object(Box1.get());
    mesh_grid_ptr->add_object(Box2.get());
    mesh_grid_ptr->setup_cells();

    world->add_object(mesh_grid_ptr);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   /* std::shared_ptr<Matte>Matt = std::make_shared<Matte>(0.20f, 0.20f, Colour{ 0.5,0.5,0.5 });
    
    world->scene.push_back(
        std::make_shared<Plane>(atlas::math::Point{ 0, 0, -1000 }, atlas::math::Vector{ 0,0,1 }));
    world->scene[1]->setMaterial(
        std::make_shared<Matte>(0.20f, 0.20f, Colour{ 0.2,0.2,0.5 }));
    
    world->scene.push_back(
        std::make_shared<Plane>(atlas::math::Point{ 0, 0, 0 }, atlas::math::Vector{ 0,1,0 }));
    world->scene[2]->setMaterial(
        std::make_shared<Matte>(0.20f, 0.20f, Colour{ 0.5,0.5,0.5 }));
     */
    //world->scene.push_back(
    //    std::make_shared<Sphere>(atlas::math::Point{ 125, 90, 150 }, 90.0f));
    //world->scene[3]->setMaterial(RSpc);

    //world->scene.push_back(
    //    std::make_shared<Cylinder>(0.0f, 250.0f, 90.0f));
    //world->scene[4]->setMaterial(GSpc);

    //world->scene.push_back(
    //    std::make_shared<Box>(atlas::math::Point{150, 0, -350}, atlas::math::Point{ 400, 250 ,-100}));
    //world->scene[5]->setMaterial(YSpc);



    /*world->scene.push_back(
        std::make_shared<Cone>(atlas::math::Point(-70, 0, 120), 150.0f, 40.0f));
    world->scene[4]->setMaterial(
        std::make_shared<Matte>(0.25f, 0.10f, Colour{ 1, 1, 0 }));*/
    //world->scene[4]->setColour({ 0, 0, 1 });
    /*
    world->scene.push_back(
        std::make_shared<Sphere>(atlas::math::Point{ -230, 85, -130 }, 85.0f));
    world->scene[5]->setMaterial(
        std::make_shared<Matte>(0.50f, 0.10f, Colour{ 0, 0, 1 }));
  
    world->scene.push_back(
        std::make_shared<Cone>(atlas::math::Point(170, 0, -190), 240.0f, 40.0f));
    world->scene[6]->setMaterial(
        std::make_shared<Matte>(0.25f, 0.10f, Colour{ 1, 0.6, 0 }));

    world->scene.push_back(
        std::make_shared<Sphere>(atlas::math::Point{ 310, 85, -30 }, 85.0f));
    world->scene[7]->setMaterial(
        std::make_shared<Matte>(0.50f, 0.10f, Colour{ 0.75, 0, 1 }));

    world->scene.push_back(
        std::make_shared<Sphere>(atlas::math::Point{ -280, 70, 40 }, 70.0f));
    world->scene[8]->setMaterial(
        std::make_shared<Matte>(0.50f, 0.10f, Colour{ 1, 0.5, 0.5}));

    world->scene.push_back(
        std::make_shared<Sphere>(atlas::math::Point{ 230, 45, 200 }, 45.0f));
    world->scene[9]->setMaterial(
        std::make_shared<Matte>(0.50f, 0.10f, Colour{ 0.5, 0, 1 }));

    world->scene.push_back(
        std::make_shared<Sphere>(atlas::math::Point{ -145, 45, 260 }, 45.0f));
    world->scene[10]->setMaterial(
        std::make_shared<Matte>(0.50f, 0.10f, Colour{ 0.5, 0.5, 1 }));

    world->scene.push_back(
        std::make_shared<Sphere>(atlas::math::Point{ 30, 50, 270}, 50.0f));
    world->scene[11]->setMaterial(
        std::make_shared<Matte>(0.50f, 0.10f, Colour{ 0, 1, 1 }));

*/
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/*Ambient Light*/
    world->ambient = std::make_shared<Ambient>();
    //set lights parameter
    world->ambient->setColour({ 1, 1, 1 });
    world->ambient->scaleRadiance(1.0f);
    

///*Ambient Occlusion*/
    //std::shared_ptr<AmbientOccluder> AmO = std::make_shared<AmbientOccluder>();
    ////AmO->scale_radiance(1.5f);
    //AmO->set_min_amount(0.2f);
    //AmO->set_sampler(new MultiJittered(4,20));
    //world->set_ambient_light(AmO);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    //world->lights.push_back(
        //std::make_shared<Directional>(Directional{ {1024, 100, 1024} }));
    
/*Point Light*/
    //std::shared_ptr pointL = std::make_shared<PointLight>();
    //pointL->setLocation({-300.0f, 350.0f, 624.0f });
    //pointL->scaleRadiance(6.5f);
    //pointL->setColor({ 1,1,1 });
    //world->add_light(pointL);   


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
    camera.setEye({ 0.0f, 350.0f, 700.0f });
    camera.setLookAt({ 0.0f, 70.0f, 0.0f });
    //camera.setZoom(9.0f);

    camera.computeUVW();
    camera.renderScene(world);
    //world->render_scene();

    

    saveToBMP("AS4.bmp", world->width, world->height, world->image);

    return 0;
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
