#pragma once

#include <atlas/math/Math.hpp>
#include <atlas/math/Ray.hpp>
#include <atlas/core/Float.hpp>
#include <atlas/math/Random.hpp>

#include <fmt/printf.h>
#include <stb_image.h>
#include <stb_image_write.h>
#include <algorithm>

#include <vector>
#include <limits>
#include <memory>

using namespace atlas;
using Colour = math::Vector;


void saveToBMP(std::string const& filename,
               std::size_t width,
               std::size_t height,
               std::vector<Colour> const& image);

// Your code here.
const float kEpsilon{ 0.01f };
const float invRAND_MAX = (float)(1.0 / (float)RAND_MAX);
const double 	PI = 3.1415926535897932384;

// Declarations
class BRDF;
class Camera;
class Material;
class Light;
class Shape;
class Sampler;
class World;
class ShadeRec;
const Colour Red = { 1,0,0 };
const Colour Green = { 0,1,0 };
const Colour Blue = { 0,0,1 };
const Colour Yellow = { 1,1,0 };

void colour_handling(Colour& c);

class Tracer {
public:
    Tracer() : world_ptr(NULL) {}
    Tracer(std::shared_ptr<World> _worldPtr) : world_ptr(_worldPtr) {}
    virtual ~Tracer(void) {
        if (world_ptr)
            world_ptr = NULL;
    }
    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const&) const=0;
    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const, const int) const = 0;
    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const, float&, const int) const = 0;
    //void print_name() {
    //    printf("%s\n", name.c_str());
    //}
protected: 
    std::shared_ptr<World> world_ptr;
    //std::string name;
};


class ViewPlane{
    public:
        int 			hres;   					// horizontal image resolution 
        int 			vres;   					// vertical image resolution
        float			s;							// pixel size
        int				image_hres;					// how big actual image should be
        int				image_vres;					// how big actual image should be
        float			gamma;						// gamma correction factor
        float			inv_gamma;					// the inverse of the gamma correction factor
        bool			show_out_of_gamut;			// display red if RGBColor out of gamut
        int				num_samples;				// number of samples per pixel
        Sampler*        sampler_ptr;				// pointer to the sampler we're using
        int				max_depth;					// How many recursive levels for reflection

    public:
        ViewPlane();   								// default Constructor
        ViewPlane(const ViewPlane& vp);				// copy constructor
        ViewPlane& operator= (const ViewPlane& rhs);		// assignment operator
        ~ViewPlane();   							// destructor
        void set_hres(const int h_res);
        void set_vres(const int v_res);
        void set_image_hres(const int h_res);
        void set_image_vres(const int v_res);
        void set_pixel_size(const float size);
        void set_gamma(const float g);
        void set_gamut_display(const bool show);
        void set_samples(const int n);
        void set_sampler(Sampler* sp);
        void set_max_depth(int depth);
};

class World
{
public:
    ViewPlane vp;
    std::size_t width, height;
    float s;							// pixel size
    Colour background;
    std::shared_ptr<Sampler> sampler;
    std::vector<std::shared_ptr<Shape>> scene; //objects
    std::vector<Colour> image;
    std::vector<std::shared_ptr<Light>> lights;
    std::shared_ptr<Light> ambient;
    std::shared_ptr<Tracer> tracer_ptr;
    std::shared_ptr<Camera> camera_ptr;
public:
    World() : background({ 0,0,0 }), tracer_ptr(NULL),
        camera_ptr(NULL), s(1.0f){}
    ~World() {};
    void add_object(std::shared_ptr<Shape> object_ptr) {
        scene.push_back(object_ptr);
    }
    void add_light(std::shared_ptr<Light> light_ptr) {
        lights.push_back(light_ptr);
    }
    void build();
    void render_scene();
    void set_camera(std::shared_ptr<Camera> camera) {
        camera_ptr = camera;
    }
    void set_ambient_light(std::shared_ptr<Light> amb) {
        if (ambient) 
            ambient = NULL;
        ambient = amb;
    }
    Colour max_to_one(const Colour& c) const {  //out of garmut control
        float max_value = std::max(c.r, std::max(c.g, c.b));
        if (max_value > 1.0)
            return (c / max_value);
        else
            return (c);
    }
    //void display_pixel(const int row, const int column, const Colour& pixel_color) const;
    ShadeRec hit_bare_bones_objects(atlas::math::Ray<atlas::math::Vector> const& ray);
    ShadeRec hit_objects(atlas::math::Ray<atlas::math::Vector> const& ray);
};

class ShadeRec
{
public:
    bool hit_an_object; //did?
    Colour color;
    float t;
    atlas::math::Point local_hit_point;
    //atlas::math::Point hit_point; // world coordinates of hit point on untransformed object (used for texture transformations)
    atlas::math::Normal normal;
    atlas::math::Ray<atlas::math::Vector> ray;
    std::shared_ptr<Material> material;
    World& world;
    int depth; //recursive
    atlas::math::Vector dir; // area lights
    float u, v; //texture cor
public:
    ShadeRec(World& wr) : hit_an_object(false), material(NULL),  local_hit_point(), normal(),
        ray(), depth(0), dir(), color({0,0,0}), world(wr), t(0.0), u(0), v(0) {}
    ShadeRec(const ShadeRec& sr) : hit_an_object(sr.hit_an_object), material(sr.material),  local_hit_point(sr.local_hit_point),
        normal(sr.normal), depth(sr.depth), dir(sr.dir), ray(sr.ray), color(sr.color), world(sr.world), t(0.0), u(sr.u), v(sr.v) {}
};

void normalize(atlas::math::Vector& v) {
    float length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    v.x /= length; v.y /= length; v.z /= length;
}
int rand_int(void) {
    return(rand());
}
float rand_float(void) {
    return((float)rand_int() * invRAND_MAX);
}

float distance(atlas::math::Point a , atlas::math::Point p) {
    return (sqrt((a.x - p.x) * (a.x - p.x)
        + (a.y - p.y) * (a.y - p.y)
        + (a.z - p.z) * (a.z - p.z)));
}

class BBox {
public:
    float x0, x1, y0, y1, z0, z1;
    BBox(void);
    BBox(const float x0, const float x1,
        const float y0, const float y1,
        const float z0, const float z1);
    BBox(const atlas::math::Point p0, const atlas::math::Point p1);
    BBox(const BBox& bbox);
    BBox& operator=(const BBox& bbox);
    BBox* clone(void) const;
    ~BBox(void);
    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray) const;
    bool inside(const atlas::math::Point& p) const;
};

// Abstract classes defining the interfaces for concrete entities
class Camera
{
public:
    Camera();
    virtual ~Camera() = default;
    virtual void renderScene(std::shared_ptr<World> world) const = 0;
    void setEye(atlas::math::Point const& eye);
    void setLookAt(atlas::math::Point const& lookAt);
    void setUpVector(atlas::math::Vector const& up);
    void computeUVW();
protected:
    atlas::math::Point mEye;
    atlas::math::Point mLookAt;
    atlas::math::Point mUp;
    atlas::math::Vector mU, mV, mW;
};


class Material
{
public:
    Material() : shadows(true) {}
    Material(const Material& m) : shadows(m.shadows) {}
    Material& operator= (const Material& m) {
        if (this == &m)
            return *this;
        shadows = m.shadows;
        return *this;
    }
    virtual std::shared_ptr<Material> clone(void) const = 0;
    virtual ~Material() = default;
    virtual Colour shade(ShadeRec&) = 0 {
        Colour bl = { 0,0,0 };
        return bl;
    }
    virtual Colour area_light_shade(ShadeRec&) {
        Colour bl = { 0,0,0 };
        return bl;
    }
    virtual Colour path_shade(ShadeRec&) {
        Colour bl = { 0,0,0 };
        return bl;
    }
    virtual Colour global_shade(ShadeRec&) {
        Colour bl = { 0,0,0 };
        return bl;
    }
    virtual Colour get_Le(ShadeRec&) const {
        Colour bl = { 0,0,0 };
        return bl;
    }
    void set_shadows(bool shadow) {
        shadows = shadow;
    }
protected:
    bool shadows;
};


class Shape
{
public:
    Shape() : mColour{ 0, 0, 0 }, mMaterial(NULL), shadows(true){}
    Shape(const Shape& object) :mColour(object.mColour), mMaterial(NULL), shadows(object.shadows)
    {}
    virtual ~Shape() = default;
    virtual std::shared_ptr<Shape> clone(void) const = 0;
    // if t computed is less than the t in sr, it and the color should be
    // updated in sr
    virtual bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& t, ShadeRec& s) const = 0;
    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray,
        float& tmin) const = 0;
    void setColour(Colour const& col) {
        mColour = col;
    }
    Colour getColour() const {
        return mColour;
    }
    void setMaterial(std::shared_ptr<Material> const& material) {
        mMaterial = material;
    }
    std::shared_ptr<Material> getMaterial() const {
        return mMaterial;
    }

    virtual atlas::math::Point sample() {
        return atlas::math::Point(0.0);
    }
    void set_shadows(const bool shadow) {
        shadows = shadow;
    }
    bool get_shadows() {
        return shadows;
    }
    virtual atlas::math::Vector get_normal(const atlas::math::Point&) const {
        return atlas::math::Vector(0.0);
    }
    virtual float pdf(const ShadeRec&) const {
        return (1.0);
    }
    virtual BBox get_bounding_box() const {
        return(BBox(-1, 1, -1, 1, -1, 1));
    }
protected:
    Colour mColour;
    std::shared_ptr<Material> mMaterial;
    bool shadows;
    Shape& operator= (const Shape& rhs) {
        if (this == &rhs)
            return (*this);
        if (rhs.mMaterial) {
            mMaterial = rhs.mMaterial->clone();
        }
        mColour = rhs.mColour;
        shadows = rhs.shadows;
        return (*this);
    }
};

class Light
{
public:
    Light(void) : shadows(true){}
    Light(const Light& ls) : shadows(ls.shadows){}
    Light& operator= (const Light& rhs) {
        if (this == &rhs)
            return (*this);
        shadows = rhs.shadows;
        return (*this);	
    }
    virtual std::shared_ptr<Light> clone() const = 0;
    virtual atlas::math::Vector getDirection(ShadeRec& sr) = 0;
    void scaleRadiance(float b);
    void setColour(Colour const& c);
    virtual float G(const ShadeRec& ) const { return 1.0; }
    virtual float pdf(const ShadeRec& ) const { return 1.0; }
    virtual Colour L(ShadeRec&) {
        Colour black{ 0,0,0 };
        return (black);
    }
    bool casts_shadows() const {
        return shadows;
    }
    virtual bool in_shadow(atlas::math::Ray<atlas::math::Vector> const& ray, const ShadeRec& sr) const = 0;
    void set_shadows(const bool shadow) {
        shadows = shadow;
    }
    ~Light(void) {}
protected:
    Colour mColour;
    float mRadiance;
    bool shadows;
};


class Sampler
{
public:
    Sampler(void) : mNumSamples(1), mNumSets(83), mCount(0), mJump(0) {
        mSamples.reserve(mNumSamples * mNumSets);
        setupShuffledIndeces();
    }
    Sampler(int numSamples, int numSets) : mNumSamples{ numSamples }, mNumSets{ numSets }, mCount{ 0 }, mJump{ 0 }
    {
        mSamples.reserve(mNumSets* mNumSamples);
        setupShuffledIndeces();
    }
    Sampler(const int num)
        : mNumSamples(num),
        mNumSets(83),
        mCount(0),
        mJump(0) {
        mSamples.reserve(mNumSamples * mNumSets);
        setupShuffledIndeces();
    }
    virtual ~Sampler() = default;
    virtual Sampler* clone(void) const = 0;
    int getNumSamples() const {
        return mNumSamples;
    }
    void setupShuffledIndeces();
    virtual void generateSamples() = 0;
    atlas::math::Point sampleUnitSquare() {
        //this is NULL ptr
        if (this == NULL) {
            printf("Line 380 NULL Sampler!\n");
        }
        if (mCount % mNumSamples == 0)
        {
            atlas::math::Random<int> engine;
            mJump = (engine.getRandomMax() % mNumSets) * mNumSamples;
        }
        return mSamples[mJump + mShuffledIndeces[mJump + mCount++ % mNumSamples]];
    }
    atlas::math::Point sample_hemisphere(void) {
        if (mCount % mNumSamples == 0)	// start of new pixel
        {
            mJump = (rand_int() % mNumSets) * mNumSamples;
        }
        return (hemisphere_samples[mJump + mShuffledIndeces[mJump + mCount++ % mNumSamples]]);
    }
    void map_samples_to_unit_disk() {
        int size = (int) mSamples.size();
        float r, phi;	//polar coordinates
        atlas::math::Point sp;	// sample point on unit disk
        disk_samples.reserve(size);
        for (int j = 0; j < size; j++) {
            // map sample point to [-1, 1]
            sp.x = (float)(2.0 * mSamples[j].x - 1.0);
            sp.y = (float)(2.0 * mSamples[j].y - 1.0);
            if (sp.x > -sp.y) {						// sectors 1 and 2
                if (sp.x > sp.y) {					// sector 1
                    r = sp.x;
                    phi = sp.y / sp.x;
                }
                else {								// sector 2
                    r = sp.y;
                    phi = 2 - sp.x / sp.y;
                }
            }
            else {									//sectors 3 and 4
                if (sp.x < sp.y) {					// sector 3
                    r = -sp.x;
                    phi = 4 + sp.y / sp.x;
                }
                else {								// sector 4
                    r = -sp.y;
                    if (sp.y != 0.0)				// BAD! / by 0 at origin
                        phi = 6 - sp.x / sp.y;
                    else
                        phi = 0.0;
                }
            }

            phi *= (float) (3.141592 / 4.0);		// common to all sectors
            disk_samples[j].x = r * cos(phi);
            disk_samples[j].y = r * sin(phi);
        }
    }
    atlas::math::Point sample_unit_disk() {
        if (mCount % mNumSamples == 0)
        {
            mJump = (rand_int() % mNumSets) * mNumSamples;
        }
        return (disk_samples[mJump + mShuffledIndeces[mJump + mCount++ % mNumSamples]]);
    }
    int get_num_samples();
    void map_samples_to_hemisphere(const float e) {
        int size = (int)mSamples.size();
        hemisphere_samples.reserve(mNumSamples * mNumSets);
        for (int j = 0; j < size; j++) {
            float cos_phi = (float)(cos(2.0 * PI * mSamples[j].x));
            float sin_phi = (float)(sin(2.0 * PI * mSamples[j].x));
            float cos_theta = (float)(pow((1.0 - mSamples[j].y), 1.0 / (e + 1.0)));
            float sin_theta = (float)(sqrt(1.0 - cos_theta * cos_theta));
            float pu = sin_theta * cos_phi;
            float pv = sin_theta * sin_phi;
            float pw = cos_theta;
            hemisphere_samples.push_back(atlas::math::Point(pu, pv, pw));
        }
    }
    //void shuffle_samples();
    void map_samples_to_sphere() {
        float r1, r2;
        float x, y, z;
        float r, phi;
        sphere_samples.reserve(mNumSamples * mNumSets);
        for (int j = 0; j < mNumSamples * mNumSets; j++) {
            r1 = mSamples[j].x;
            r2 = mSamples[j].y;
            z = (float)(1.0 - 2.0 * r1);
            r = (float)(sqrt(1.0 - z * z));
            phi = (float)(2.0* PI * r2);
            x = r * cos(phi);
            y = r * sin(phi);
            sphere_samples.push_back(atlas::math::Point(x, y, z));
        }
    }
    atlas::math::Point sample_sphere() {
        //printf("L461\n");
        //printf("Size of Indeces:%d\n", (int)(sizeof(sphere_samples)));
        if (mNumSamples == NULL)
            printf("mCount NULL!\n");
        //printf("L461 mCount:%d, mNumSamples:%d\n", mCount, mNumSamples);
        
        if (mCount % mNumSamples == 0)  									
            mJump = (rand_int() % mNumSets) * mNumSamples;
        return (sphere_samples[mJump + mShuffledIndeces[mJump + mCount++ % mNumSamples]]);
    }

protected:
    std::vector<atlas::math::Point> mSamples;
    std::vector <atlas::math::Point> disk_samples;
    std::vector <atlas::math::Point> hemisphere_samples;
    std::vector <atlas::math::Point> sphere_samples;
    std::vector<int> mShuffledIndeces;
    int mNumSamples;
    int mNumSets;
    unsigned long mCount;
    int mJump;
};



// ***Samplers***
class Regular : public Sampler
{
public:
    Regular(int numSamples, int numSets);
    Regular::Regular(const int num)
        : Sampler(num)
    {
        generateSamples();
    }
    Regular* Regular::clone(void) const {
        return (new Regular(*this));
    }
    void generateSamples();
};

class Random : public Sampler
{
public:
    Random(int numSamples, int numSets);
    Random* Random::clone(void) const {
        return (new Random(*this));
    }
    void generateSamples();
};

class Jittered : public Sampler
{
public:
    Jittered(int numSamples, int numSets);
    Jittered* Jittered::clone(void) const {
        return (new Jittered(*this));
    }
    void generateSamples();
};

class MultiJittered : public Sampler
{
public:
    MultiJittered(const int num_samples)
        : Sampler(num_samples) {
        generateSamples();
    }
    MultiJittered(const int num_samples, const int m);
    MultiJittered* MultiJittered::clone(void) const {
        return (new MultiJittered(*this));
    }

    void generateSamples();
};



class BRDF
{
public:
    BRDF() : sampler_ptr(){}
    BRDF(const BRDF& brdf) {
        if (brdf.sampler_ptr) {
            sampler_ptr = brdf.sampler_ptr->clone(); 
        }
        else {
            sampler_ptr = NULL;
        }
    }
    virtual BRDF* clone()const = 0;
    BRDF& operator= (const BRDF& rhs) {
        if (this == &rhs)
            return (*this);
        if (sampler_ptr) {
            //delete sampler_ptr;
            sampler_ptr = NULL;
        }
        if (rhs.sampler_ptr) {
            sampler_ptr = rhs.sampler_ptr->clone();	
        }
        return (*this);
    }
    virtual ~BRDF() = default;
    void set_sampler(Sampler* sPtr) {
        sampler_ptr = sPtr;
        sampler_ptr->map_samples_to_hemisphere(1);	// perfect diffuse
    }
    virtual Colour fn(ShadeRec const& sr,
        atlas::math::Vector const& reflected,
        atlas::math::Vector const& incoming) const = 0;
    virtual Colour rho(ShadeRec const& sr,
        atlas::math::Vector const& reflected) const = 0;
    virtual Colour sample_f(const ShadeRec& sr, atlas::math::Vector& wi, const atlas::math::Vector& wo) const = 0;
protected:
    Sampler* sampler_ptr;
};




// Concrete classes which we can construct and use in our ray tracer
// ***Cameras***
class Pinhole : public Camera
{
public:
    Pinhole();
    void setDistance(float distance);
    void setZoom(float zoom);
    atlas::math::Vector rayDirection(atlas::math::Point const& p) const;
    void renderScene(std::shared_ptr<World> world) const;

private:
    float mDistance;
    float mZoom;
};


// ***Shapes***
class Sphere : public Shape
{
public:
    Sphere(void) : Shape(), center(0.0), radius(1.0), inv_area((float)(1 / (4 * PI * exp2(radius)))),
        sampler_ptr(NULL){}
    Sphere(atlas::math::Point center, float radius) :
        center{ center }, radius{ radius }
    {}
    virtual std::shared_ptr<Shape> clone() const {
        return (std::make_shared<Sphere>(*this));
    }
    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& tmin, ShadeRec& sr) const {
        double 	t;
        atlas::math::Vector	temp = ray.o - center;
        double 	a = glm::dot(ray.d , ray.d);
        double 	b = 2.0 * glm::dot(temp ,ray.d);
        double 	c = glm::dot(temp ,temp) - radius * radius;
        double 	disc = b * b - 4.0 * a * c;
        if (disc < 0.0)
            return(false);
        else {
            double e = sqrt(disc);
            double denom = 2.0 * a;
            t = (-b - e) / denom;    // smaller root
            if (t > kEpsilon) {
                tmin = t;
                sr.normal = (temp + (float)t * ray.d) / (float)radius;
                sr.local_hit_point = ray.o + (float)t * ray.d;
                return (true);
            }
            t = (-b + e) / denom;    // larger root
            if (t > kEpsilon) {
                tmin = t;
                sr.normal = (temp + (float)t * ray.d) / (float)radius;
                sr.local_hit_point = ray.o + (float)t * ray.d;
                return (true);
            }
        }
        return (false);
    }
    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray, float& tmin) const {
        if (!shadows)
            return false;
        double t;
        atlas::math::Vector	temp = ray.o - center;
        double a = glm::dot(ray.d , ray.d);
        double b = 2.0 * glm::dot( temp , ray.d);
        double c = glm::dot(temp, temp) - radius * radius;
        double disc = b * b - 4.0 * a * c;
        if (disc < 0.0)
            return(false);
        else {
            double e = sqrt(disc);
            double denom = 2.0 * a;
            t = (-b - e) / denom;    // smaller root
            if (t > kEpsilon) {
                tmin = (float)t;
                return (true);
            }
            t = (-b + e) / denom;    // larger root

            if (t > kEpsilon) {
                tmin = (float)t;
                return (true);
            }
        }
        return (false);
    }
    void set_sampler(Sampler* sampler) {
        if (sampler_ptr) {
            // free(sampler_ptr);
            sampler_ptr = NULL;
        }
        sampler_ptr = sampler;
        sampler_ptr->map_samples_to_sphere();
    }
    atlas::math::Point sample() {
        atlas::math::Point sample_point = sampler_ptr->sample_sphere();
        //printf("sample_point return{%f,%f,%f}\n", sample_point.x, sample_point.y, sample_point.z);
        return (center + (float)(sample_point.x * radius) + (float)(sample_point.y * radius) + (float)(sample_point.z * radius));
    }
    float pdf(ShadeRec& ) const {
        return (inv_area);
    }
    virtual atlas::math::Vector get_normal(const atlas::math::Point& p) const {
        atlas::math::Point v = (center - p);
        glm::normalize(v);
        return v;				
    }
    virtual BBox get_bounding_box() const {
        double delta = 0.0001;
        return (BBox(atlas::math::Point(center.x - radius - delta, center.y - radius - delta, center.z - radius - delta),
            atlas::math::Point(center.x + radius + delta, center.y + radius + delta, center.z + radius + delta)));
    }
private:
    atlas::math::Point center;
    double 	radius;
    Sampler* sampler_ptr;		// sampler for being a light object
    float	inv_area;
};

class Disk : public Shape {
public:
    Disk() : Shape(), center(1.0), r_squared(9), radius(3), normal({ 0, 0, 1 }) {}
    Disk(atlas::math::Point c, float r, atlas::math::Vector n) : Shape(), center(c), r_squared(pow(r, 2)), radius(r), normal(n)
    {}
    Disk(const Disk& d) : Shape(d), center(d.center), r_squared(d.r_squared), radius(d.radius),
        normal(d.normal) {}
    Disk& operator=(const Disk& d) {
        if (this == &d)
            return (*this);
        Shape::operator= (d);
        center = d.center;
        r_squared = d.r_squared;
        normal = d.normal;
        radius = d.radius;
        return (*this);
    }
    virtual std::shared_ptr<Shape> clone(void) const {
        return(std::make_shared<Disk>(*this));
    }
    virtual bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& tmin, ShadeRec& sr) const {
        float t = (float)glm::dot((center - ray.o) , normal) / glm::dot(ray.d , normal);

        if (t <= kEpsilon)
            return (false);
        atlas::math::Point p = ray.o + t * ray.d;
        float d_suqare = (float)((center.x - p.x) * (center.x - p.x)
            + (center.y - p.y) * (center.y - p.y)
            + (center.z - p.z) * (center.z - p.z));
        if (d_suqare < r_squared) {
            tmin = t;
            sr.normal = normal;
            sr.local_hit_point = p;
            return (true);
        }
        else
            return (false);
    }

    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray, float& tmin) const {
        float t = (float)glm::dot((center - ray.o) , normal) / glm::dot(ray.d , normal);
        if (t <= kEpsilon)
            return (false);
        atlas::math::Point p = ray.o + t * ray.d;
        float d_suqare = (float)((center.x - p.x) * (center.x - p.x)
            + (center.y - p.y) * (center.y - p.y)
            + (center.z - p.z) * (center.z - p.z));
        if (d_suqare < r_squared) {
            tmin = t;
            //		sr.normal = normal;
            //		sr.local_hit_point = p;
            return (true);
        }
        else
            return (false);
    }
    float d_squared(atlas::math::Point p) {
        //return (pow((p.distance(center)), 2));
        return (pow(((sqrt((p.x - center.x) * (p.x - center.x)
            + (p.y - center.y) * (p.y - center.y)
            + (p.z - center.z) * (p.z - center.z)))), 2));
        
    }
    virtual BBox get_bounding_box(void) const {
        double delta = 0.0001;
        // This is Circle's bounding box, could be more efficient
        return (BBox(atlas::math::Point(center.x - radius - delta, center.y - radius - delta, center.z - radius - delta),
            atlas::math::Point(center.x + radius + delta, center.y + radius + delta, center.z + radius + delta)));
    }

private:
    atlas::math::Point center;
    float r_squared;
    float radius;		// for bbox
    atlas::math::Vector normal;
};

class Rectangle : public Shape {
public:
    Rectangle(void) : Shape(),
        p0(-1, 0, -1),
        a(0, 0, 2), b(2, 0, 0),
        a_len_squared(4.0),
        b_len_squared(4.0),
        normal(0, 1, 0),
        area(4.0),
        inv_area(0.25),
        sampler_ptr(NULL)
    {}
    Rectangle(const atlas::math::Point& _p0, const atlas::math::Vector& _a, const atlas::math::Vector& _b) : Shape(),
        p0(_p0),
        a(_a),
        b(_b),
        a_len_squared(_a.x * _a.x + _a.y * _a.y + _a.z * _a.z),
        b_len_squared(_b.x* _b.x + _b.y * _b.y + _b.z * _b.z),
        area((float)((a.length()* b.length()))),
        inv_area((float)(1.0 / area)),
        sampler_ptr(NULL)
    {
        //normal = a ^ b;
        normal = atlas::math::Vector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
        normalize(normal);
    }

    Rectangle(const atlas::math::Point& _p0, const atlas::math::Vector& _a, const atlas::math::Vector& _b, const atlas::math::Vector& n) : Shape(),
        p0(_p0),
        a(_a),
        b(_b),
        a_len_squared(_a.x* _a.x + _a.y * _a.y + _a.z * _a.z),
        b_len_squared(_b.x* _b.x + _b.y * _b.y + _b.z * _b.z),
        area((float)(a.length()* b.length())),
        inv_area((float)(1.0 / area)),
        normal(n),
        sampler_ptr(NULL)
    {
        normalize(normal);
    }

    virtual std::shared_ptr<Shape>
        clone(void) const {
        return (std::make_shared<Rectangle>(*this));
    }

    Rectangle(const Rectangle& r) : Shape(r),
        p0(r.p0),
        a(r.a),
        b(r.b),
        a_len_squared(r.a_len_squared),
        b_len_squared(r.b_len_squared),
        normal(r.normal),
        area(r.area),
        inv_area(r.inv_area)
    {
        if (r.sampler_ptr)
            sampler_ptr = r.sampler_ptr->clone();
        else  sampler_ptr = NULL;
    }

    virtual
        ~Rectangle(void) {

        if (sampler_ptr) {
            delete sampler_ptr;
            sampler_ptr = NULL;
        }
    }

    Rectangle&
        operator= (const Rectangle& rhs) {
        if (this == &rhs)
            return (*this);

        Shape::operator=(rhs);

        p0 = rhs.p0;
        a = rhs.a;
        b = rhs.b;
        a_len_squared = rhs.a_len_squared;
        b_len_squared = rhs.b_len_squared;
        area = rhs.area;
        inv_area = rhs.inv_area;
        normal = rhs.normal;

        if (sampler_ptr) {
            delete sampler_ptr;
            sampler_ptr = NULL;
        }

        if (rhs.sampler_ptr)
            sampler_ptr = rhs.sampler_ptr->clone();

        return (*this);
    }

    virtual BBox
        get_bounding_box(void) const {
        double delta = 0.0001;

        return(BBox((float)std::min(p0.x, p0.x + a.x + b.x) - (float)delta, (float)std::max(p0.x, p0.x + a.x + b.x) + (float)delta,
            (float)std::min(p0.y, p0.y + a.y + b.y) - (float)delta, (float)std::max(p0.y, p0.y + a.y + b.y) + (float)delta,
            (float)std::min(p0.z, p0.z + a.z + b.z) - (float)delta, (float)std::max(p0.z, p0.z + a.z + b.z) + (float)delta));
    }

    virtual bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& tmin, ShadeRec& sr) const {

        double t = glm::dot((p0 - ray.o) , normal) /glm::dot (ray.d , normal);
        if (t <= kEpsilon)
            return (false);
        atlas::math::Point p = ray.o + (float)t * ray.d;
        atlas::math::Vector d = p - p0;
        double ddota = glm::dot(d , a);
        if (ddota < 0.0 || ddota > a_len_squared)
            return (false);
        double ddotb = glm::dot(d , b);
        if (ddotb < 0.0 || ddotb > b_len_squared)
            return (false);
        tmin = t;
        sr.normal = normal;
        sr.local_hit_point = p;

        return (true);
    }

    // the following functions are used when the rectangle is a light source
    virtual void set_sampler(Sampler* sampler) {
        if (sampler_ptr) {
            delete sampler_ptr;
            sampler_ptr = NULL;
        }

        sampler_ptr = sampler;
    }
    virtual atlas::math::Point sample(void) {
        //printf("Lin 927 find NULL ptr\n");
        if (sampler_ptr == NULL)
            printf("Line 929 Null at Shape\n");
        //sampler_ptr = new Regular(1, 1);
        atlas::math::Point sample_point = sampler_ptr->sampleUnitSquare(); //sampler_ptr is NULL
        return (p0 + sample_point.x * a + sample_point.y * b);
    }
    virtual atlas::math::Vector
        get_normal(const atlas::math::Point& ) const {
        return (normal);
    }
    virtual float pdf(ShadeRec&) const {
        return (inv_area);
    }
    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray, float& tmin) const {
        if (!shadows)
            return false;
        double t = glm::dot((p0 - ray.o), normal) / glm::dot(ray.d, normal);
        if (t <= kEpsilon)
            return (false);
        atlas::math::Point p = ray.o + (float)t * ray.d;
        atlas::math::Vector d = p - p0;
        double ddota = glm::dot(d, a);
        if (ddota < 0.0 || ddota > a_len_squared)
            return (false);
        double ddotb =glm::dot(d, b);

        if (ddotb < 0.0 || ddotb > b_len_squared)
            return (false);

        tmin = (float)t;
        //sr.normal 			= normal;
        //sr.local_hit_point 	= p;

        return (true);
    }

private:

    atlas::math::Point 		p0;   			// corner vertex 
    atlas::math::Vector		a;				// side
    atlas::math::Vector		b;				// side
    double			a_len_squared;	// square of the length of side a
    double			b_len_squared;	// square of the length of side b
    atlas::math::Vector	normal;

    float			area;			// for rectangular lights
    float			inv_area;		// for rectangular lights
    Sampler* sampler_ptr;	// for rectangular lights 	
};


class Compound : public Shape {
public:
    Compound() : Shape() {}
    virtual std::shared_ptr<Shape> clone(void) const {
        return(std::make_shared<Compound>(*this));
    }
    Compound(const Compound& c) : Shape(c) {
        copy_objects(c.objects);
    }
    ~Compound(void) {
        delete_objects();
    }
    Compound& operator= (const Compound& c) {
        if (this == &c)
            return (*this);
        Shape::operator= (c);
        copy_objects(c.objects);
        return (*this);
    }
    virtual void set_material(std::shared_ptr<Material> mat_ptr) {
        int num_objects = (int)objects.size();
        for (int j = 0; j < num_objects; j++)
            objects[j]->setMaterial(mat_ptr);
    }
    virtual void add_object(Shape* obj_ptr) {
        objects.push_back(obj_ptr);
    }

    int get_num_objects(void) {
        return ((int)objects.size());
    }

    virtual bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& tmin, ShadeRec& sr) const {
        double t;
        atlas::math::Vector normal{ 0,0,0 };
        atlas::math::Point local_hit_point{ 0,0,0 };
        bool hit = false;
        tmin = 1.0E10;
        int num_objects = (int)objects.size();

        for (int j = 0; j < num_objects; j++)
            if (objects[j]->hit(ray, t, sr) && (t < tmin)) {
                hit = true;
                tmin = t;
                //material_ptr = objects[j]->getMaterial();
                normal = sr.normal;
                local_hit_point = sr.local_hit_point;
            }

        if (hit) {
            sr.t = (float)tmin;
            sr.normal = normal;
            sr.local_hit_point = local_hit_point;
        }

        return (hit);
    }

    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray, float& tmin) const {
        float t;
        //	Normal normal;
        //	atlas::math::Point local_hit_point;
        bool hit = false;
        tmin = 1.0E10;
        int num_objects = (int)objects.size();

        for (int j = 0; j < num_objects; j++)
            if (objects[j]->shadow_hit(ray, t) && (t < tmin)) {
                hit = true;
                tmin = t;
                //material_ptr	= objects[j]->get_material();
                //normal			= sr.normal;
                //local_hit_point = sr.local_hit_point;
            }

        //	if (hit) {
        //		sr.t				= tmin;
        //		sr.normal			= normal;
        //		sr.local_hit_point	= local_hit_point;
        //	}

        return (hit);
    }

protected:
    std::vector<Shape*> objects;

private:
    void delete_objects(void) {
        int num_objects = (int)objects.size();
        for (int j = 0; j < num_objects; j++) {
            if (objects[j]) {
                //delete objects[j];
                objects[j] = NULL;
            }
        }
        objects.erase(objects.begin(), objects.end());
    }
    void copy_objects(const std::vector<Shape*>& rhs_objects) {
        delete_objects();
        int num_objects = (int)rhs_objects.size();
        for (int j = 0; j < num_objects; j++)
            objects.push_back((rhs_objects[j]->clone()).get());
    }

};



class OpenCylinder : public Shape {
public:
    OpenCylinder(void) : Shape(), y0(-1.0), y1(1.0), radius(1.0), inv_radius(1.0)
    {}
    OpenCylinder(const double bottom, const double top, const double radius) : Shape(), y0(bottom), y1(top), radius(radius),
        inv_radius(1.0 / radius) {}
    OpenCylinder(const OpenCylinder& c) : Shape(c), y0(c.y0), y1(c.y1),  radius(c.radius),
        inv_radius(c.inv_radius) {}
    virtual std::shared_ptr<Shape> clone() const {
        return(std::make_shared<OpenCylinder>(*this));
    }
    OpenCylinder& operator= (const OpenCylinder& c) {
        if (this == &c)
            return (*this);
        Shape::operator= (c);
        y0 = c.y0;
        y1 = c.y1;
        radius = c.radius;
        inv_radius = c.inv_radius;
        return (*this);
    }
    virtual ~OpenCylinder(void) {}
    virtual bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& tmin, ShadeRec& sr) const {
        double t;
        double ox = ray.o.x;
        double oy = ray.o.y;
        double oz = ray.o.z;
        double dx = ray.d.x;
        double dy = ray.d.y;
        double dz = ray.d.z;
        double a = dx * dx + dz * dz;
        double b = 2.0 * (ox * dx + oz * dz);
        double c = ox * ox + oz * oz - radius * radius;
        double disc = b * b - 4.0 * a * c;
        if (disc < 0.0)
            return(false);
        else {
            double e = sqrt(disc);
            double denom = 2.0 * a;
            t = (-b - e) / denom;    // smaller root
            if (t > kEpsilon) {
                double yhit = oy + t * dy;
                if (yhit > y0&& yhit < y1) {
                    tmin = t;
                    sr.normal = atlas::math::Vector((ox + t * dx) * inv_radius, 0.0, (oz + t * dz) * inv_radius);
                    // test for hitting from inside
                    if (-glm::dot(ray.d , sr.normal) < 0.0)
                        sr.normal = -sr.normal;
                    sr.local_hit_point = ray.o + (float)tmin * ray.d;
                    return (true);
                }
            }

            t = (-b + e) / denom;    // larger root

            if (t > kEpsilon) {
                double yhit = oy + t * dy;
                if (yhit > y0&& yhit < y1) {
                    tmin = t;
                    sr.normal = atlas::math::Vector((ox + t * dx) * inv_radius, 0.0, (oz + t * dz) * inv_radius);
                    // test for hitting inside surface
                    if (-glm::dot(ray.d , sr.normal) < 0.0)
                        sr.normal = -sr.normal;
                    sr.local_hit_point = ray.o + (float)tmin * ray.d;
                    return (true);
                }
            }
        }
        return (false);
    }
    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray, float& tmin) const {

        double t;
        double ox = ray.o.x;
        double oy = ray.o.y;
        double oz = ray.o.z;
        double dx = ray.d.x;
        double dy = ray.d.y;
        double dz = ray.d.z;
        double a = dx * dx + dz * dz;
        double b = 2.0 * (ox * dx + oz * dz);
        double c = ox * ox + oz * oz - radius * radius;
        double disc = b * b - 4.0 * a * c;
        if (disc < 0.0)
            return(false);
        else {
            double e = sqrt(disc);
            double denom = 2.0 * a;
            t = (-b - e) / denom;    // smaller root
            if (t > kEpsilon) {
                double yhit = oy + t * dy;
                if (yhit > y0&& yhit < y1) {
                    tmin = (float)t;
                    //sr.normal = Normal((ox + t * dx) * inv_radius, 0.0, (oz + t * dz) * inv_radius);
    //				// test for hitting from inside
    //				if (-ray.d * sr.normal < 0.0)
    //					sr.normal = -sr.normal;
    //				sr.local_hit_point = ray.o + tmin * ray.d;
                    return (true);
                }
            }
            t = (-b + e) / denom;    // larger root
            if (t > kEpsilon) {
                double yhit = oy + t * dy;
                if (yhit > y0&& yhit < y1) {
                    tmin = (float) t;
                    //				sr.normal = Normal((ox + t * dx) * inv_radius, 0.0, (oz + t * dz) * inv_radius);	
                    //				// test for hitting inside surface
                    //				
                    //				if (-ray.d * sr.normal < 0.0)
                    //					sr.normal = -sr.normal;
                    //				
                    //				sr.local_hit_point = ray.o + tmin * ray.d;
                    return (true);
                }
            }
        }
        return (false);
    }
    virtual BBox get_bounding_box(void) const {
        double delta = 0.0001;
        return (BBox(atlas::math::Point(-radius - delta, y0 - delta, -radius - delta),
            atlas::math::Point(radius + delta, y1 + delta, radius + delta)));
    }
protected:

    double y0;				// bottom
    double y1;				// top
    double radius;
    double inv_radius;		// 1 / radius
};

class Plane : public Shape
{
public:
    Plane(void) : Shape(), point_on_plane(0.0), normal(0, 1, 0){}
    Plane(atlas::math::Point p, atlas::math::Vector n) :
        point_on_plane{ p }, normal{ n }
    {}
    virtual std::shared_ptr<Shape> clone() const {
        return (std::make_shared <Plane>(*this));
    }
    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& tmin, ShadeRec& sr) const {
        float t = glm::dot((point_on_plane - ray.o) , normal) / glm::dot(ray.d , normal);
        if (t > kEpsilon) {
            tmin = t;
            sr.normal = normal;
            sr.local_hit_point = ray.o + t * ray.d;
            return (true);
        }
        return(false);
    }
    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray, float& tmin) const {
        if (!shadows)
            return false;
        float t = glm::dot((point_on_plane - ray.o) , normal) / glm::dot(ray.d , normal);
        if (t > kEpsilon) {
            tmin = t;
            return (true);
        }
        return(false);
    }
private:
    atlas::math::Point point_on_plane;
    atlas::math::Vector normal;
};

class Box : public Shape
{
public:
    Box() : x0(-100), x1(100), y0(-100), y1(100), z0(-100), z1(100){}
    Box(atlas::math::Point p0, atlas::math::Point p1) : x0(p0.x), x1(p1.x), y0(p0.y), y1(p1.y), z0(p0.z), z1(p1.z)
    {}
    std::shared_ptr<Shape> clone() const {
        return (std::make_shared <Box>(*this));
    }
    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& tmin, ShadeRec& sr) const {
        double ox = ray.o.x;
        double oy = ray.o.y;
        double oz = ray.o.z;
        double dx = ray.d.x;
        double dy = ray.d.y;
        double dz = ray.d.z;
        double tx_min, ty_min, tz_min;
        double tx_max, ty_max, tz_max;
        double a = 1.0 / dx;
        if (a >= 0) {
            tx_min = (x0 - ox) * a;
            tx_max = (x1 - ox) * a;
        }
        else {
            tx_min = (x1 - ox) * a;
            tx_max = (x0 - ox) * a;
        }
        double b = 1.0 / dy;
        if (b >= 0) {
            ty_min = (y0 - oy) * b;
            ty_max = (y1 - oy) * b;
        }
        else {
            ty_min = (y1 - oy) * b;
            ty_max = (y0 - oy) * b;
        }
        double c = 1.0 / dz;
        if (c >= 0) {
            tz_min = (z0 - oz) * c;
            tz_max = (z1 - oz) * c;
        }
        else {
            tz_min = (z1 - oz) * c;
            tz_max = (z0 - oz) * c;
        }
        double t0, t1;

        int face_in, face_out;

        if (tx_min > ty_min) {
            t0 = tx_min;
            face_in = (a >= 0.0) ? 0 : 3;
        }
        else {
            t0 = ty_min;
            face_in = (b >= 0.0) ? 1 : 4;
        }

        if (tz_min > t0) {
            t0 = tz_min;
            face_in = (c >= 0.0) ? 2 : 5;
        }

 
        if (tx_max < ty_max) {
            t1 = tx_max;
            face_out = (a >= 0.0) ? 3 : 0;
        }
        else {
            t1 = ty_max;
            face_out = (b >= 0.0) ? 4 : 1;
        }

        if (tz_max < t1) {
            t1 = tz_max;
            face_out = (c >= 0.0) ? 5 : 2;
        }

        if (t0 < t1 && t1 > kEpsilon) {  // condition for a hit
            if (t0 > kEpsilon) {
                tmin = t0;  			// ray hits outside surface
                sr.normal = get_normal(face_in);
            }
            else {
                tmin = t1;				// ray hits inside surface
                sr.normal = get_normal(face_out);
            }

            sr.local_hit_point = ray.o + (float)tmin * ray.d;
            return (true);
        }
        else
            return (false);

    }
    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray, float& tmin) const {
        if (!shadows)
            return false;
        float ox = ray.o.x;
        float oy = ray.o.y;
        float oz = ray.o.z;
        float dx = ray.d.x;
        float dy = ray.d.y;
        float dz = ray.d.z;
        float tx_min, ty_min, tz_min;
        float tx_max, ty_max, tz_max;
        float a = (float)(1.0 / dx);
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
        int face_in, face_out;
        // find largest entering t value

        if (tx_min > ty_min) {
            t0 = tx_min;
            face_in = (a >= 0.0) ? 0 : 3;
        }
        else {
            t0 = ty_min;
            face_in = (b >= 0.0) ? 1 : 4;
        }

        if (tz_min > t0) {
            t0 = tz_min;
            face_in = (c >= 0.0) ? 2 : 5;
        }

        // find smallest exiting t value

        if (tx_max < ty_max) {
            t1 = tx_max;
            face_out = (a >= 0.0) ? 3 : 0;
        }
        else {
            t1 = ty_max;
            face_out = (b >= 0.0) ? 4 : 1;
        }

        if (tz_max < t1) {
            t1 = tz_max;
            face_out = (c >= 0.0) ? 5 : 2;
        }

        if (t0 < t1 && t1 > kEpsilon) {  // condition for a hit
            if (t0 > kEpsilon) {
                tmin = t0;  			// ray hits outside surface
                //sr.normal = get_normal(face_in);
            }
            else {
                tmin = t1;				// ray hits inside surface
                //sr.normal = get_normal(face_out);
            }

            //sr.local_hit_point = ray.o + tmin * ray.d;
            return (true);
        }
        else
            return (false);
    }
    atlas::math::Vector get_normal(const int face_hit) const {
        switch (face_hit) {
            case 0:       return (atlas::math::Vector(-1, 0, 0)); // -x face
            case 1:       return (atlas::math::Vector(0, -1, 0)); // -y face
            case 2:       return (atlas::math::Vector(0, 0, -1)); // -z face
            case 3:       return (atlas::math::Vector(1, 0, 0));  // +x face
            case 4:       return (atlas::math::Vector(0, 1, 0));  // +y face
            case 5:       return (atlas::math::Vector(0, 0, 1));  // +z face
            default:    return(atlas::math::Vector(0, 0, 0));
        }
    }
    BBox get_bounding_box(void) const {
        return (BBox(x0, x1, y0, y1, z0, z1));
    }
private:
  
    //std::vector<atlas::math::Point> vertices;
    float x0, x1, y0, y1, z0, z1;
};
/*
class OpenCylinder : public Shape {
public:
    OpenCylinder(const float bottom, const float top, const float radius);
    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, ShadeRec& tracer) const;
protected:
    float		y0;				// bottom y value
    float		y1;				// top y value
    float		radius;			// radius
    float		inv_radius;  	// one over the radius	
    bool intersectRay(atlas::math::Ray<atlas::math::Vector> const&, float&) const { return true; }
    bool intersectRay(atlas::math::Ray<atlas::math::Vector> const& ray, ShadeRec& sr) const;
};
*/

class Cylinder : public Compound
{
public:
    // See implementations for comments
    Cylinder() : Compound(), bbox(atlas::math::Point(-1, -1, -1), atlas::math::Point(1, 1, 1)) {
        objects.push_back(new Disk(atlas::math::Point(0, -1, 0), 1, atlas::math::Vector(0, -1, 0)));
        objects.push_back(new Disk(atlas::math::Point(0, 1, 0), 1, atlas::math::Vector(0, 1, 0)));
        objects.push_back(new OpenCylinder(-1, 1, 1));
    }
    Cylinder(const float bottom, const float top, const float radius) : Compound(),
        bbox(atlas::math::Point(-radius, bottom, -radius), atlas::math::Point(radius, top, radius)) {
        objects.push_back(new Disk(atlas::math::Point(0, bottom, 0), radius, atlas::math::Vector(0, -1, 0)));
        objects.push_back(new Disk(atlas::math::Point(0, top, 0), radius, atlas::math::Vector(0, 1, 0)));
        objects.push_back(new OpenCylinder(bottom, top, radius));
    }

    Cylinder(const Cylinder& cc) : Compound(cc), bbox(cc.bbox) {}
    std::shared_ptr<Shape> clone() const {
        return (std::make_shared <Cylinder>(*this));
    }
    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double& tmin, ShadeRec& sr) const {
        if (bbox.hit(ray))
            return (Compound::hit(ray, tmin, sr));
        else
            return (false);
    }
    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& ray, float& tmin) const {
        if (bbox.hit(ray))
            return (Compound::shadow_hit(ray, tmin));
        else
            return (false);
    }
    virtual BBox get_bounding_box(void) const {
        return bbox;
    }
protected:
    BBox bbox;

};

class Cone : public Shape
{
public:
    Cone(const atlas::math::Point& c, float h, float r) : center(c), height(h), radius(r) { calculateAngle(); }
    std::shared_ptr<Shape> clone() const {
        return (std::make_shared <Cone>(*this));
    }
    atlas::math::Vector normal_in(const atlas::math::Point& p) const {
        atlas::math::Point c0(center.x, p.y, center.z);
        atlas::math::Point c1(center.x, center.y + height, center.z);
        atlas::math::Vector v = p - center;
        v.y = 0;
        normalize(v);
        atlas::math::Vector n;
        n.x = v.x * height / radius;
        n.z = v.z * height / radius;
        n.y = radius / height;
        return n;
    }
    void calculateAngle() // calculate the angle at the cone vertex
    {
        atlas::math::Point p1(center.x + radius, center.y, center.z);
        atlas::math::Point p2(center.x, center.y + height, center.z);
        float d = (float)((p2 - p1).length()*1.0);
        cosAlphaSq = height / d;
        sinAlphaSq = radius / d;
    }
    bool hit(atlas::math::Ray<atlas::math::Vector> const& ray, double&tMin, ShadeRec& sr) const {
        float t{ std::numeric_limits<float>::max() };
        bool intersect{ intersectRay(ray, t) };
        // update ShadeRec info about new closest hit
        if (intersect && t < sr.t)
        {
            sr.local_hit_point = ray.o + t * ray.d;
            sr.normal = normal_in(atlas::math::Vector{ ray.o.x + t * ray.d.x, ray.o.y + t * ray.d.y, ray.o.z + t * ray.d.z });
            sr.ray = ray;
            sr.color = mColour;
            sr.t = t;
            sr.material = mMaterial;
            tMin = t;
        }

        return intersect;
    }
    virtual bool shadow_hit(atlas::math::Ray<atlas::math::Vector> const& , float&) const {
            return false;
    }

protected:
    bool intersectRay(atlas::math::Ray<atlas::math::Vector> const& ray, float& tMin) const {
        atlas::math::Point p0(ray.o.x - center.x, ray.o.y - center.y - height, ray.o.z - center.z);
        // got them mathematically intersecting the line equation with the cone equation
        float a = cosAlphaSq * ray.d.x * ray.d.x + cosAlphaSq * ray.d.z * ray.d.z - sinAlphaSq * ray.d.y * ray.d.y;
        float b = cosAlphaSq * ray.d.x * p0.x + cosAlphaSq * ray.d.z * p0.z - sinAlphaSq * ray.d.y * p0.y;
        float c = cosAlphaSq * p0.x * p0.x + cosAlphaSq * p0.z * p0.z - sinAlphaSq * p0.y * p0.y;
        float delta = b * b - a * c;
        // delta < 0 means no intersections
        if (delta < kEpsilon)
            return false;
        // nearest intersection
        tMin = (-b - sqrt(delta)) / a;
        // t<0 means the intersection is behind the ray origin
        if (tMin < kEpsilon)
            return false;
        float y = p0.y + tMin * ray.d.y;
        if (y < -height - kEpsilon || y > kEpsilon)
            return false;
        return true;
    }
    float cosAlphaSq; //top angle
    float sinAlphaSq;
    atlas::math::Point center; // center of the base
    float height; // height of the cone
    float radius;//radius of the base
};



// ***BRDF***
class Lambertian : public BRDF
{
public:
    Lambertian();
    Lambertian(Colour diffuseColor, float diffuseReflection);
    BRDF* Lambertian::clone(void) const {
        return (new Lambertian(*this));
    }
    Colour fn(ShadeRec const& sr,
        atlas::math::Vector const& reflected,
        atlas::math::Vector const& incoming) const override;
    Colour rho(ShadeRec const& sr,
        atlas::math::Vector const& reflected) const override;
    void setDiffuseReflection(float kd);
    void setDiffuseColour(Colour const& colour);
    void set_kd(const float the_kd) {
        mDiffuseReflection = the_kd;
    }
    void set_cd(const Colour cd) {
        mDiffuseColour = cd;
    }
    Colour Lambertian::sample_f(const ShadeRec&, atlas::math::Vector& , const atlas::math::Vector& ) const {
        Colour bl = { 0,0,0 };
        return (bl);
    }
    Colour Lambertian::sample_f(const ShadeRec& sr, atlas::math::Vector& wi, const atlas::math::Vector&, float& pdf) const {
        atlas::math::Vector w = sr.normal;
        //atlas::math::Vector v = atlas::math::Vector(0.0034, 1.0, 0.0071) ^ w;
        atlas::math::Vector v = atlas::math::Vector(1.0 * w.z - 0.0071 * w.y, 0.0071 * w.x - 0.0034 * 0.0071, 0.0034 * w.y - 1.0 * w.x);
        //return (atlas::math::vector(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x));

        glm::normalize(v);
       // atlas::math::Vector u = v ^ w;
        atlas::math::Vector u = atlas::math::Vector(v.y * w.z - v.z * w.y, v.z * w.x - v.x * w.z, v.x * w.y - v.y * w.x);
        if (sampler_ptr == NULL)
            printf("null sampler in line 1081\n");
        atlas::math::Point sp = sampler_ptr->sample_hemisphere();
        printf("line 1081\n");
        wi = sp.x * u + sp.y * v + sp.z * w;
        normalize(wi);
        pdf = (float)(glm::dot(sr.normal , wi) * glm::one_over_pi<float>());
        
        return (mDiffuseColour * mDiffuseReflection * glm::one_over_pi<float>());
    }
private:
    Colour mDiffuseColour;      //cd
    float mDiffuseReflection;   //kd
};


class PerfectSpecular : public BRDF {
public:
    PerfectSpecular() : BRDF(), kr(1.0), cr(Colour(1.0, 1.0, 1.0)){}
    PerfectSpecular(const PerfectSpecular& lamb) : BRDF(lamb),  kr(lamb.kr), cr(lamb.cr) {}
    PerfectSpecular& operator= (const PerfectSpecular& rhs) {
        if (this == &rhs)
            return (*this);
        BRDF::operator= (rhs);
        kr = rhs.kr;
        cr = rhs.cr;
        return (*this);
    }
    ~PerfectSpecular(void) {}
    virtual BRDF* clone(void) const {
        return (new PerfectSpecular(*this));
    }
    void set_kr(const float the_kr) {
        kr = the_kr;
    }
    void set_cr(const Colour the_cr) {
        cr = the_cr;
    }
    void set_cr(const float r, const float g, const float b) {
        cr.r = r; cr.g = g; cr.b = b;
    }
    virtual Colour fn(const ShadeRec& sr, const atlas::math::Vector& wi, const atlas::math::Vector& wo) const {
        //return (kr * cr * invPI);
        return BRDF::fn(sr, wi, wo);
    }
    virtual Colour sample_f(const ShadeRec& sr, atlas::math::Vector& wi, const atlas::math::Vector& wo) const {
        float ndotwo =(float)glm::dot( sr.normal , wo);
        wi = (-wo + (float)2.0 * sr.normal * ndotwo);
        return (kr * cr / (sr.normal * wi));
        // For Transparent material
        //return (kr * cr / fabs(sr.normal * wi));
    }
    virtual Colour sample_f(const ShadeRec& sr, atlas::math::Vector& wi, const atlas::math::Vector& wo, float& pdf) const {
        float ndotwo = (float) glm::dot(sr.normal , wo);
        wi = -wo + (float)2.0 * sr.normal * ndotwo;
        pdf = (float)glm::dot(sr.normal , wi);
        return (kr * cr);
    }
    virtual Colour rho(const ShadeRec& sr, const atlas::math::Vector& wo) const {
        //return (kr * cr);
        return BRDF::rho(sr, wo);
    }
private:

    float kr;	// reflectance coefficient
    Colour cr;	// reflectance color
};

class GlossySpecular : public BRDF {
public:
    GlossySpecular() : BRDF(), ks(0.0), cs(1.0), exp(1.0), sampler_ptr(NULL) {}
    GlossySpecular(const GlossySpecular& lamb) : BRDF(lamb), ks(lamb.ks), cs(lamb.cs), exp(lamb.exp)
    {
        if (lamb.sampler_ptr)
            sampler_ptr = lamb.sampler_ptr->clone();
        else
            sampler_ptr = NULL;
    }
    GlossySpecular& operator= (const GlossySpecular& rhs) {
        if (this == &rhs)
            return (*this);
        BRDF::operator= (rhs);
        ks = rhs.ks;
        cs = rhs.cs;
        exp = rhs.exp;
        if (sampler_ptr) {
            //delete sampler_ptr;
            sampler_ptr = NULL;
        }
        if (rhs.sampler_ptr)
            sampler_ptr = rhs.sampler_ptr->clone();
        return (*this);
    }
    virtual BRDF* clone(void) const {
        return (new GlossySpecular(*this));
    }
    void set_ks(const float the_ks) {
        ks = the_ks;
    }
    void
        set_exp(const float the_exp) {
        exp = the_exp;
    }
    void
        set_cs(const Colour the_cs) {
        cs = the_cs;
    }
    void set_cs(const float r, const float g, const float b) {
        cs.r = r; cs.g = g, cs.b = b;
    }
    virtual Colour fn(const ShadeRec& sr, const atlas::math::Vector& wi, const atlas::math::Vector& wo) const {
        Colour L = { 0,0,0 };
        float wi_length = (float)sqrt(wi.x * wi.x + wi.y * wi.y + wi.z * wi.z);
        atlas::math::Vector wiUnit = wi / wi_length;
        //float ndotwi = (float)glm::dot(sr.normal , wi);
        //atlas::math::Vector r(-wi + (float)2.0 * sr.normal * ndotwi);
        float ndotwi = (float)glm::dot(sr.normal, wiUnit);
        atlas::math::Vector r(-wiUnit + (float)2.0 * sr.normal * ndotwi);
        float rdotwo = (float)glm::dot(r , wo);////
        if (rdotwo > 0.0) {
            L += (float)(ks * pow(rdotwo, exp));
            //printf("L:(%f,%f,%f)\n", L.r, L.g, L.b);
        }
        return (L);
    }
    // Not used, but abstract without
    virtual Colour sample_f(const ShadeRec& , atlas::math::Vector& , const atlas::math::Vector& ) const {
        Colour black{ 0,0,0 };
        return (black);
    }
    virtual Colour sample_f(const ShadeRec& sr, atlas::math::Vector& wi, const atlas::math::Vector& wo, float& pdf) const {
        float ndotwo = (float)glm::dot(sr.normal, wo);
        Colour black{ 0,0,0 };
        atlas::math::Vector r = -wo + (float)2.0 * sr.normal * ndotwo;     // direction of mirror reflection
        atlas::math::Vector w = r;
       //atlas::math::Vector u = atlas::math::Vector(0.00424, 1, 0.00764) ^ w;
        atlas::math::Vector u = atlas::math::Vector(1 * w.z - 0.00764 * w.y, 0.00764 * w.x - 0.00424 * w.z, 0.00424 * w.y - 1 * w.x);
        normalize(u);
       //atlas::math::Vector v = u ^ w;
        atlas::math::Vector v = atlas::math::Vector(u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x);

        atlas::math::Point sp = sampler_ptr->sample_hemisphere();
        wi = sp.x * u + sp.y * v + sp.z * w;			// reflected ray direction
        if (glm::dot(sr.normal , wi) < 0.0) 						// reflected ray is below tangent plane
            wi = -sp.x * u - sp.y * v + sp.z * w;
        float phong_lobe = (float)pow(glm::dot(r , wi), exp);
        pdf = (float)(phong_lobe * glm::dot(sr.normal , wi));

        return (ks * cs * phong_lobe);
    }
    virtual Colour rho(const ShadeRec& , const atlas::math::Vector& ) const {
        Colour black{ 0,0,0 };
        return (black);
    }
    void set_samples(const int num_sample, const float exps) {
        sampler_ptr = new MultiJittered(num_sample);
        sampler_ptr->map_samples_to_hemisphere(exps);
    }
    void set_normal(const atlas::math::Vector& n);

private:

    float ks;			// specular coefficient
    Colour cs;		// specular color
    float exp;			// specular exponent
    Sampler* sampler_ptr;
};






// ***Materials***
class Matte : public Material
{
public:
    Matte() : Material{}, mDiffuseBRDF{ new Lambertian() }, mAmbientBRDF{ new Lambertian() }{}
    Matte(float kd, float ka, Colour color);
    Matte(const Matte& m) : Material(m) {
        if (m.mAmbientBRDF)
            mAmbientBRDF = (Lambertian*)(m.mAmbientBRDF->clone());
        else
            mAmbientBRDF = NULL;

        if (m.mDiffuseBRDF)
            mDiffuseBRDF = (Lambertian*)(m.mDiffuseBRDF->clone());
        else
            mAmbientBRDF = NULL;
    }
    virtual std::shared_ptr<Material> clone() const {
        return (std::make_shared<Matte>(*this));
    }
    void set_ka(const float ka) {
        mAmbientBRDF->set_kd(ka);
    }
    void set_kd(const float kd) {
        mDiffuseBRDF->set_kd(kd);
    }
    void set_cd(const Colour& c) {
        mAmbientBRDF->set_cd(c);
        mDiffuseBRDF->set_cd(c);
    }
    void set_sampler(Sampler* sampl_ptr) {
        mAmbientBRDF->set_sampler(sampl_ptr);
        mDiffuseBRDF->set_sampler(sampl_ptr->clone());
    }
    virtual Colour shade(ShadeRec& sr);
    virtual Colour area_light_shade(ShadeRec& sr) {
        atlas::math::Vector wo = -sr.ray.d;
        //Colour L{ 0,0,0 };
        Colour L = (mAmbientBRDF->rho(sr, wo) * sr.world.ambient->L(sr));
        int num_lights = (int)(sr.world.lights.size());
        for (int j = 0; j < num_lights; j++) {
            //printf("light.normal\n");
            atlas::math::Vector wi = sr.world.lights[j]->getDirection(sr);
            //printf("light.normal: (%f,%f,%f)\n", wi.x, wi.y, wi.z);
            //printf("sr.normal: (%f,%f,%f)\n", sr.normal.x, sr.normal.y, sr.normal.z);
            float ndotwi = (float)(glm::dot(sr.normal, wi));
            //printf("ndotwi:%f\n", ndotwi);
            if (ndotwi > 0.0f) {
                bool in_shadow = false;
                if (!shadows)
                    ;
                else {
                    in_shadow = false;
                    if (sr.world.lights[j]->casts_shadows()) {  
                        //Ray shadow_ray(sr.hit_point, wi);
                        atlas::math::Ray<atlas::math::Vector> shadow_ray;
                        shadow_ray.o =sr.local_hit_point;
                        shadow_ray.d = wi;
                        in_shadow = sr.world.lights[j]->in_shadow(shadow_ray, sr);
                    }
                }
                if (!in_shadow) {
                    //printf("not in shadow\n");
                    L += mDiffuseBRDF->fn(sr, wo, wi) * sr.world.lights[j]->L(sr) * sr.world.lights[j]->G(sr) * ndotwi / sr.world.lights[j]->pdf(sr);
                }
            }
        }
        return (L);
    }
    virtual Colour path_shade(ShadeRec& sr) {
        atlas::math::Vector wi;
        atlas::math::Vector wo = -sr.ray.d;
        float pdf;
        //printf("in path shade\n");
        //mDiffuseBRDF->set_sampler((Sampler*)(sr.world->sampler).get());
        Colour f = mDiffuseBRDF->sample_f(sr, wi, wo, pdf);
        //printf("in path shade colour f\n");
        float ndotwi = glm::dot(sr.normal, wi);
        atlas::math::Ray<atlas::math::Vector> reflected_ray(sr.local_hit_point, wi);
        return (f * sr.world.tracer_ptr->trace_ray(reflected_ray, sr.depth + 1) * ndotwi / pdf);
    }
    virtual Colour global_shade(ShadeRec& sr);

    
    void setDiffuseReflection(float k);
    void setAmbientReflection(float k);
    void setDiffuseColour(Colour colour);
    //Colour shade(ShadeRec& sr) override;
private:
    Lambertian* mDiffuseBRDF;
    Lambertian* mAmbientBRDF;
};

class Emissive : public Material
{
public:
    Emissive() : Material(), ls(1.0), ce({ 1.0,1.0,1.0 }) {}
    Emissive(const Emissive& em) : Material(em), ls(em.ls), ce(em.ce) {}
    Emissive& operator=(const Emissive& em) {
        if (this == &em)
            return (*this);
        ls = em.ls;
        ce = em.ce;
        return (*this);
    }
    //~Emissive();
    virtual std::shared_ptr<Material> clone() const {
        return (std::make_shared <Emissive>(*this));
    }
    void scale_radiance(const float b) {
        ls = b;
    }
    void set_ce(const float r, const float g, const float b) {
        ce.r = r; ce.g = g; ce.b = b;
    }
    void set_ce(const Colour& c) {
        ce.r = c.r; ce.g = c.g; ce.b = c.b;
    }
    virtual Colour get_Le(ShadeRec& ) const {
        return (ls * ce);
    }
    virtual Colour shade(ShadeRec& sr) {
        Colour black = { 0,0,0 };
        if (glm::dot(-sr.normal , sr.ray.d) > 0.0)
            return (ls * ce);
        else
            return (black);
    }
    virtual Colour area_light_shade(ShadeRec& sr) {
        Colour black = { 0,0,0 };
        if (glm::dot(-sr.normal, sr.ray.d) > 0.0)
            return (ls * ce);
        else
            return (black);
    }
    virtual Colour global_shade(ShadeRec& sr) {
        Colour black = { 0,0,0 };
        if (sr.depth == 1)
            return (black);

        if (glm::dot(-sr.normal, sr.ray.d) > 0.0)
            return (ls * ce);
        else
            return (black);
    }
    virtual Colour path_shade(ShadeRec& sr) {
        Colour black = { 0,0,0 };
        if (glm::dot(-sr.normal, sr.ray.d) > 0.0)
            return (ls * ce);
        else
            return (black);
    }

private:
    float ls;
    Colour ce;
};


class Phong : public Material {
public:

    Phong(void) : Material(),
        ambient_brdf(new Lambertian),
        diffuse_brdf(new Lambertian),
        specular_brdf(new GlossySpecular)
    {}

    Phong(const Phong& rhs) : Material(rhs) {
        if (rhs.ambient_brdf)
            ambient_brdf = (Lambertian*)(rhs.ambient_brdf->clone());
        else
            ambient_brdf = NULL;

        if (rhs.diffuse_brdf)
            diffuse_brdf = (Lambertian*)(rhs.diffuse_brdf->clone());
        else
            diffuse_brdf = NULL;

        if (rhs.specular_brdf)
            specular_brdf = (GlossySpecular*)(rhs.specular_brdf->clone());
        else
            specular_brdf = NULL;
    }

    Phong& operator= (const Phong&p) {
        if (this == &p)
            return (*this);
        Material::operator= (p);
        if (ambient_brdf) {
            //delete ambient_brdf;
            ambient_brdf = NULL;
        }
        if (diffuse_brdf) {
            //delete diffuse_brdf;
            diffuse_brdf = NULL;
        }
        if (specular_brdf) {
            //delete specular_brdf;
            specular_brdf = NULL;
        }
        if (p.ambient_brdf)
            ambient_brdf = (Lambertian*)(p.ambient_brdf->clone());
        if (p.diffuse_brdf)
            diffuse_brdf = (Lambertian*)(p.diffuse_brdf->clone());
        if (p.specular_brdf)
            specular_brdf = (GlossySpecular*)(p.specular_brdf->clone());
    }
    virtual std::shared_ptr<Material> clone() const {
        return (std::make_shared<Phong>(*this));
    }
    virtual Colour shade(ShadeRec& sr) {
        atlas::math::Vector wo = -sr.ray.d;
        Colour L = ambient_brdf->rho(sr, wo) * sr.world.ambient->L(sr);
        int num_lights = (int)sr.world.lights.size();

        for (int j = 0; j < num_lights; j++) {
            atlas::math::Vector wi = sr.world.lights[j]->getDirection(sr);
            float ndotwi = (float)glm::dot(sr.normal , wi);

            if (ndotwi > 0.0) {

                bool in_shadow = false;
                //
                if (sr.world.lights[j]->casts_shadows()) {
                    //atlas::math::Ray<atlas::math::Vector> shadowRay(sr.hit_point, wi);
                    atlas::math::Ray<atlas::math::Vector> shadowRay;
                    shadowRay.o = sr.local_hit_point;
                    shadowRay.d = wi;
                    in_shadow = sr.world.lights[j]->in_shadow(shadowRay, sr);
                }

                if (!in_shadow) {
                    L += (diffuse_brdf->fn(sr, wi, wo) + specular_brdf->fn(sr, wi, wo)) * sr.world.lights[j]->L(sr)* ndotwi;
                }
            }
        }
        return (L);
    }
    void set_ka(const float ka) {
        if (ambient_brdf)
            ambient_brdf->set_kd(ka);
    }
    void set_kd(const float kd) {
        if (diffuse_brdf)
            diffuse_brdf->set_kd(kd);
    }
    void set_ks(const float ks) {
        if (specular_brdf)
            specular_brdf->set_ks(ks);
    }
    void set_ca(const float r, const float g, const float b) {
        if (ambient_brdf)
            ambient_brdf->set_cd({ r, g, b });
    }
    void set_cd(const float r, const float g, const float b) {
        if (diffuse_brdf)
            diffuse_brdf->set_cd({ r, g, b });
    }
    void set_cd(const Colour& c) {
        set_cd(c.r, c.b, c.g);
    }
    void set_cs(const float r, const float g, const float b) {
        if (specular_brdf)
            specular_brdf->set_cs(r, g, b);
    }
    void set_exp_s(const float exp_s) {
        if (specular_brdf)
            specular_brdf->set_exp(exp_s);
    }
    void set_exp(const float exp_s) {
        set_exp_s(exp_s);
    }

    void set_k(const float k) {
        if (ambient_brdf)
            ambient_brdf->set_kd(k);
        if (diffuse_brdf)
            diffuse_brdf->set_kd(k);
        if (specular_brdf)
            specular_brdf->set_ks(k);
    }
    void set_c(const float r, const float g, const float b) {
        if (ambient_brdf)
            ambient_brdf->set_cd({ r, g, b });
        if (diffuse_brdf)
            diffuse_brdf->set_cd({ r, g, b });
        if (specular_brdf)
            specular_brdf->set_cs({ r, g, b });
    }
    void set_c(const Colour& c) {
        if (ambient_brdf)
            ambient_brdf->set_cd(c);

        if (diffuse_brdf)
            diffuse_brdf->set_cd(c);

        if (specular_brdf)
            specular_brdf->set_cs(c);
    }
    virtual Colour area_light_shade(ShadeRec& sr) {
        atlas::math::Vector wo = -sr.ray.d;
        Colour L = ambient_brdf->rho(sr, wo) * sr.world.ambient->L(sr);
        int num_lights = (int)sr.world.lights.size();

        for (int j = 0; j < num_lights; j++) {
            atlas::math::Vector wi = sr.world.lights[j]->getDirection(sr);
            float ndotwi = (float)glm::dot(sr.normal , wi);

            if (ndotwi > 0.0) {
                bool in_shadow = false;
                if (sr.world.lights[j]->casts_shadows()) {
                    atlas::math::Ray<atlas::math::Vector> shadow_ray(sr.local_hit_point, wi);
                    in_shadow = sr.world.lights[j]->in_shadow(shadow_ray, sr);
                }

                if (!in_shadow) {
                    L += (diffuse_brdf->fn(sr, wi, wo) + specular_brdf->fn(sr, wi, wo)) * sr.world.lights[j]->L(sr) *
                        sr.world.lights[j]->G(sr) * ndotwi /
                        sr.world.lights[j]->pdf(sr);
                    if (specular_brdf->fn(sr, wi, wo).r > 0.001 || specular_brdf->fn(sr, wi, wo).g > 0.001 || specular_brdf->fn(sr, wi, wo).b > 0.001) {
                       // printf("Spec_brdf-f = (%f,%f,%f)\n", specular_brdf->fn(sr, wi, wo).r, specular_brdf->fn(sr, wi, wo).g, specular_brdf->fn(sr, wi, wo).b);

                      //  printf("lights[%d]->getDirection(sr) = (%f,%f,%f)\n", j, wi.x, wi.y, wi.z);
                    }
                    
                }
            }
        }
        return (L);
    }

private:
    Lambertian* ambient_brdf;
    Lambertian* diffuse_brdf;
    GlossySpecular* specular_brdf;
};


// ***Lights***
class Directional : public Light
{
public:
    Directional();
    Directional(const Directional& pl);
    virtual std::shared_ptr<Light> clone() const {
        return (std::make_shared<Directional>(*this));
    }
    void setDirection(atlas::math::Vector const& d);
    atlas::math::Vector getDirection(ShadeRec& sr) override;
    virtual Colour L(ShadeRec& ) {
        return (ls * color);
    }
    virtual bool in_shadow(atlas::math::Ray<atlas::math::Vector> const&ray, const ShadeRec&sr) const {
        float t;
        int num_objects = (int)sr.world.scene.size();
        //float d = location.distance(ray.o);

        for (int j = 0; j < num_objects; j++)
            if (sr.world.scene[j]->shadow_hit(ray, t))// && t < d)
                return (true);

        return (false);
    }
private:
    float ls;
    Colour color;
    atlas::math::Vector direction;
};

class Ambient : public Light
{
public:
    Ambient() : Light(), ls(1.0), color(1.0) {}
    Ambient(const Ambient& amb) : Light(amb), ls(amb.ls), color(amb.color) {}
    Ambient& operator= (const Ambient& amb) {
        if (this == &amb)
            return (*this);
        Light::operator= (amb);
        ls = amb.ls;
        color = amb.color;
        return (*this);
    }
    virtual std::shared_ptr<Light> clone() const {
        return (std::make_shared<Ambient>(*this));
    }
    void scale_radiance(const float b) {
        ls = b;
    }
    void set_color(const float c) {
        color.r = c; color.g = c; color.b = c;
    }
    void set_color(const Colour& c) {
        color = c;
    }

    void set_color(const float r, const float g, const float b) {
        color.r = r; color.g = g; color.b = b;
    }
    virtual Colour L(ShadeRec& ) {
        return (ls * color);
    }
    atlas::math::Vector getDirection(ShadeRec& ) override {
        return (atlas::math::Vector(0.0));				
    }
    virtual bool in_shadow(atlas::math::Ray<atlas::math::Vector> const&, const ShadeRec&) const { return false; }
protected:
    float ls;
    Colour color;
};


class AmbientOccluder : public Ambient {
public:
    AmbientOccluder(void) : Ambient(), sampler_ptr(NULL) {}
    AmbientOccluder(const AmbientOccluder& amb) : Ambient(amb),
        sampler_ptr(amb.sampler_ptr->clone()),
        min_amount(Colour((float)0.001)),
        u(amb.u), v(amb.v), w(amb.w) {}
    AmbientOccluder& operator= (const AmbientOccluder& amb) {
        if (this == &amb)
            return (*this);
        Ambient::operator= (amb);
        if (amb.sampler_ptr) {
            if (sampler_ptr) {
                delete sampler_ptr;
                sampler_ptr = NULL;
            }
            sampler_ptr = amb.sampler_ptr->clone();
        }
        u = amb.u;
        v = amb.v;
        w = amb.w;
        min_amount = amb.min_amount;
        return (*this);
    }
    ~AmbientOccluder() {}
    void set_sampler(Sampler* s_ptr) {
        if (sampler_ptr) {
            //delete sampler_ptr;
            sampler_ptr = NULL;
        }
        sampler_ptr = s_ptr;
        sampler_ptr->map_samples_to_hemisphere(1);
    }
    virtual atlas::math::Vector get_direction(ShadeRec& ) {
        atlas::math::Point sp = sampler_ptr->sample_hemisphere();
        return (sp.x * u + sp.y * v + sp.z * w);
    }
    virtual bool in_shadow(atlas::math::Ray<atlas::math::Vector> const& ray, const ShadeRec& sr) const {
        float t;
        int num_objects = (int)sr.world.scene.size();
        for (int j = 0; j < num_objects; j++) {
            if (sr.world.scene[j]->shadow_hit(ray, t))
                return (true);
        }
        return (false);
    }
    virtual Colour L(ShadeRec& sr) {
        w = sr.normal;
        // jitter the up vector, otherwise normal being vertical will be a problem
        //v = w ^ atlas::math::Vector(0.0072, 1.0, 0.0034);
        v = atlas::math::Vector(w.y * 0.0034 - w.z * 1.0, w.z * 0.0072 - w.x * 0.0034, w.x * 1.0 - w.y * 0.0072);
        normalize(v);
        //u = v ^ w;
        u = atlas::math::Vector(v.y * w.z - v.z * w.y, v.z * w.x - v.x * w.z, v.x * w.y - v.y * w.x);
        atlas::math::Ray<atlas::math::Vector> shadow_ray;
        shadow_ray.o = sr.local_hit_point;
        shadow_ray.d = get_direction(sr);
        if (in_shadow(shadow_ray, sr))
            return (min_amount * ls * color);
        else
            return (ls * color);
    }
    void set_min_amount(const float min) {
        min_amount.r = min_amount.g = min_amount.b = min;
    }
private:
    atlas::math::Vector u, v, w;			// Used to form an orthonormal basis
    Sampler* sampler_ptr;		// To sample the hemisphere around a point
    Colour min_amount;		// Used to shade parts that are occluded
	//float ls;					// light brightness
};


class PointLight : public Light
{
public:
    PointLight() : Light(), ls(1.0), color(1.0), location(0) {}
    PointLight(const PointLight& pl) : Light(pl), ls(pl.ls), color(pl.color), location(pl.location) {}
    PointLight& operator= (const PointLight& pl) {
        if (this == &pl)
            return (*this);
        Light::operator= (pl);
        location = pl.location;
        mColour = pl.mColour;
        mRadiance = pl.mRadiance;
        ls = pl.ls;
        color = pl.color;
        return (*this);
    }
    virtual std::shared_ptr<Light> clone() const {
        return (std::make_shared<PointLight>(*this));
    }
    atlas::math::Vector getDirection(ShadeRec& sr) {
        atlas::math::Vector temp = location;
        atlas::math::Vector  temp2 = location - sr.local_hit_point;
        float length = (float)sqrt(temp2.x * temp2.x + temp2.y * temp2.y + temp2.z * temp2.z);
        atlas::math::Vector  temp3 = temp2 / length;
        return (temp3);
    }
    void scaleRadiance(const float b) {
        ls = b;
    }
    void setColor(const Colour& c) {
        color = c;
    }
    void setLocation(atlas::math::Point p) {
        location.x = p.x; location.y = p.y; location.z = p.z;
    }
    virtual Colour L(ShadeRec& ) {
        return (ls * color);
    }
    virtual bool in_shadow(atlas::math::Ray<atlas::math::Vector> const& ray, const ShadeRec& sr) const {
        float t;
        int num_objects = (int)sr.world.scene.size();
        float d = sqrt((location.x - ray.o.x) * (location.x - ray.o.x) + (location.y - ray.o.y) * (location.y - ray.o.y) + (location.z - ray.o.z) * (location.z - ray.o.z));     //distance;
        for (int j = 0; j < num_objects; j++)
            if (sr.world.scene[j]->shadow_hit(ray, t) && t < d)
                return (true);

        return (false);
    }

private:
    atlas::math::Point location;
    float ls;
    Colour color;
};



class FakeSphericalLight : public Light {
public:
    FakeSphericalLight();
    FakeSphericalLight(atlas::math::Point const& p) {
        location = p;
    }
    FakeSphericalLight(const FakeSphericalLight& pl);
    FakeSphericalLight& operator= (const FakeSphericalLight& pl);
    virtual std::shared_ptr<Light> clone() const {
        return (std::make_shared<FakeSphericalLight>(*this));
    }
    void scaleRadiance(const float b) {
        ls = b;
    }
    void setColor(const Colour& c) {
        color = c;
    }
    void setLocation(const atlas::math::Point& p) {
        location = p;
    }
    virtual atlas::math::Vector getDirection(ShadeRec& sr);
    virtual Colour L(ShadeRec&) {
        return (ls* color);
    }
    virtual bool in_shadow(atlas::math::Ray<atlas::math::Vector> const &ray, const ShadeRec& sr) const;
    void setRadius(const float radius) {
        r = radius;
    }

private:
    float r;		// radius
    float ls;
    Colour color;
    atlas::math::Point location;
};


// AreaLight
class AreaLight : public Light {
public:
    AreaLight() : Light(), object_ptr(NULL), material_ptr(NULL){}
    AreaLight(const AreaLight& al) : Light(al) {
        if (al.object_ptr)
            object_ptr = al.object_ptr->clone();
        else
            object_ptr = NULL;

        if (al.material_ptr)
            material_ptr = al.material_ptr->clone();
        else
            material_ptr = NULL;
    }
    AreaLight& operator= (const AreaLight& al) {
        if (this == &al)
            return (*this);
        if (object_ptr) {
            //delete object_ptr;
            object_ptr = NULL;
        }
        if (material_ptr) {
            material_ptr = NULL;
        }
        if (al.object_ptr)
            object_ptr = al.object_ptr->clone();
        if (al.material_ptr)
            material_ptr = al.material_ptr->clone();
        return (*this);
    }
    //~AreaLight(void);
    virtual std::shared_ptr<Light> clone() const {
        return (std::make_shared<AreaLight>(*this));
    }
    virtual atlas::math::Vector getDirection(ShadeRec& sr) {
        if(object_ptr!=NULL)
       
        sample_point = object_ptr->sample();
       // printf("Line 2115 get sample\n");
        light_normal = object_ptr->get_normal(sample_point);
        //printf("Line2115 Are Normal:(%f,%f,%f)\n", light_normal.x,light_normal.y, light_normal.z);
        wi = sample_point - sr.local_hit_point;  
        return (wi);
    }
    virtual bool in_shadow(atlas::math::Ray<atlas::math::Vector> const& ray, const ShadeRec& sr) const {
        float t;
        int num_objects = (int)(sr.world.scene.size());
        float ts = (float)(glm::dot((sample_point - ray.o) , ray.d));
        //float ts = sqrt((sample_point.x - ray.o.x) * (sample_point.x - ray.o.x) + (sample_point.y - ray.o.y) * (sample_point.y - ray.o.y) + (sample_point.z - ray.o.z) * (sample_point.z - ray.o.z));
        //printf("ts = %f\n", ts);  //ts is the problem
        //ts = (float)0.0;
        for (int j = 0; j < num_objects; j++)
            if (sr.world.scene[j]->shadow_hit(ray, t) && t < ts)
                return true;
        //printf("Not in shadow!\n");
        return (false);
    }

    virtual Colour L(ShadeRec& sr) {
        float ndotd = -glm::dot(light_normal , wi);
        Colour black = { 0,0,0 };
        if (ndotd > 0.0)
            return (material_ptr->get_Le(sr));
        else
            return (black);
    }
    virtual float G(const ShadeRec& sr) const {
        float ndotd = -glm::dot(light_normal, wi);
        float d2 = (float)((sample_point.x - sr.local_hit_point.x) * (sample_point.x - sr.local_hit_point.x)
            + (sample_point.y - sr.local_hit_point.y) * (sample_point.y - sr.local_hit_point.y)
            + (sample_point.z - sr.local_hit_point.z) * (sample_point.z - sr.local_hit_point.z));
        return (ndotd / d2);
    }
    virtual float pdf(const ShadeRec& sr) const {
        return (object_ptr->pdf(sr));
    }
    void set_object(std::shared_ptr<Shape> obj_ptr) {
        object_ptr = obj_ptr;
        material_ptr = object_ptr->getMaterial();
    }
private:
    std::shared_ptr<Shape> object_ptr;
    std::shared_ptr<Material> material_ptr;			// emissive material
    atlas::math::Point sample_point;			// sample point on surface
    atlas::math::Vector light_normal;			// normal at sample point
    atlas::math::Vector wi;					// unit vector from hit point to sample point

};


class RayCast : public Tracer {
public:
    RayCast() : Tracer() {}
    RayCast(std::shared_ptr<World> _worldPtr) : Tracer(_worldPtr) {}
    virtual ~RayCast() {}
    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const ray, const int) const {
        ShadeRec sr(world_ptr->hit_objects(ray));
        if (sr.hit_an_object) {
            sr.ray = ray;			// used for specular shading
            return (sr.material->shade(sr));
        }
        else
            return (world_ptr->background);
    }
    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const& ray) const override {
        ShadeRec sr(world_ptr->hit_objects(ray));
        if (sr.hit_an_object) {
            sr.ray = ray;			// used for specular shading
            return (sr.material->shade(sr));
        }
        else
            return (world_ptr->background);
    }
    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const ray, float&, const int) const override {
        ShadeRec sr(world_ptr->hit_objects(ray));
        if (sr.hit_an_object) {
            sr.ray = ray;			// used for specular shading
            return (sr.material->shade(sr));
        }
        else
            return (world_ptr->background);
    }
    
};

class AreaLighting : public Tracer {
public:
    AreaLighting() : Tracer() {}
    AreaLighting(std::shared_ptr<World> _world_ptr) : Tracer(_world_ptr) {}
    ~AreaLighting() {}
    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const& ray) const override {
        ShadeRec sr(world_ptr->hit_objects(ray)); // sr is copy constructed
        if (sr.hit_an_object) {
            sr.ray = ray;
            return (sr.material->area_light_shade(sr));
        }
        else{
            return (world_ptr->background);
        }
    }
    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const ray, const int depth) const override {
        ShadeRec sr(world_ptr->hit_objects(ray)); // sr is copy constructed
        if (sr.hit_an_object) {
            //printf("trace_ray hit\n");
            sr.depth = depth;
            sr.ray = ray; 
            Colour temp = sr.material->area_light_shade(sr);
            //printf("(%f,%f,%f)\n", temp.r, temp.g, temp.b);
            //return(temp);
            return (sr.material->area_light_shade(sr));
        }
        else
            return (world_ptr->background);
    }

    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const ray, float& tmin, const int depth) const override {
        Colour black = { 0,0,0 };
        if (depth > world_ptr->vp.max_depth)
            return (black);
        else {
            ShadeRec sr(world_ptr->hit_objects(ray));

            if (sr.hit_an_object) {
                sr.depth = depth;
                sr.ray = ray;
                tmin = sr.t;     // for colored transparency 
                return (sr.material->area_light_shade(sr));
            }
            else {
                tmin = (float)1.0E10;
                return (world_ptr->background);
            }
        }
    }
};

class Whitted : public Tracer {
public:

    Whitted() : Tracer(){}
    Whitted(std::shared_ptr<World> _world_ptr) : Tracer(_world_ptr) {}
    ~Whitted() {}

    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const& ray) const override {
        ShadeRec sr(world_ptr->hit_objects(ray)); // sr is copy constructed
        if (sr.hit_an_object) {
            sr.ray = ray;
            return (sr.material->shade(sr));
        }
        else
            return (world_ptr->background);
    }

    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const ray, const int depth) const override {
        if (depth > world_ptr->vp.max_depth) {
            Colour black = { 0,0,0 };
            return (black);
        }
        else {
            ShadeRec sr(world_ptr->hit_objects(ray)); 
            if (sr.hit_an_object) {
                sr.depth = depth;
                sr.ray = ray;
                return (sr.material->shade(sr));
            }
            else
                return (world_ptr->background);
        }
    }

    virtual Colour trace_ray(atlas::math::Ray<atlas::math::Vector> const ray, float& tmin, const int depth) const override{
        if (depth > world_ptr->vp.max_depth) {
            Colour black = { 0,0,0 };
            return (black);
        }
        else {
            ShadeRec sr(world_ptr->hit_objects(ray));
            if (sr.hit_an_object) {
                sr.depth = depth;
                sr.ray = ray;
                tmin = sr.t;     // required for colored transparency 
                return (sr.material->shade(sr));
            }
            else {
                tmin = (float)1.0E10;
                return (world_ptr->background);
            }
        }
    }

};


void process_indicator(const int r, const int vres) {
    if (r == (int)(vres / 10))
        printf("Process: 10%%\n");
    else if (r == (int)(vres / 5))
        printf("Process: 20%%\n");
    else if (r == (int)(vres / 3.33))
        printf("Process: 30%%\n");
    else if (r == (int)(vres / 2))
        printf("Process: 50%%\n");
    else if (r == (int)(vres / 1.67))
        printf("Process: 60%%\n");
    else if (r == (int)(vres / 1.538))
        printf("Process: 65%%\n");
    else if (r == (int)(vres / 1.43))
        printf("Process: 70%%\n");
    else if (r == (int)(vres / 1.33))
        printf("Process: 75%%\n");
    else if (r == (int)(vres / 1.25))
        printf("Process: 80%%\n");
    else if (r == (int)(vres / 1.176))
        printf("Process: 85%%\n");
    else if (r == (int)(vres / 1.11))
        printf("Process: 90%%\n");
    else if (r == (int)(vres / 1.053))
        printf("Process: 95%%\n");
}