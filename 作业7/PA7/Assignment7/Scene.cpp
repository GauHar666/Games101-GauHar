//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

//判断光线是否与当前场景中的某个包围盒相交，求一条光线与场景的交点
Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

//对场景中光源进行随机采样
void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0; //发光区域面积
    //遍历场景中所有物体
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){ //如果发光
            emit_area_sum += objects[k]->getArea(); //累加物体的发光面积
        }
    }
    float p = get_random_float() * emit_area_sum; //生成随机数，表示在发光区域中等概率随机采样一个点
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf); //当找到发光物体值大于p时，利用Sample函数采样一个位置。
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f L_dir;
    Vector3f L_indir;

    //从像素发射出光线
    Intersection obj_inter = intersect(ray);
    if(!obj_inter.happened) return L_dir;//没发生相交

    //如果打到光源
    if(obj_inter.m->hasEmission()) return obj_inter.m->getEmission();

    //打到物体
    Vector3f p = obj_inter.coords;//交点坐标
    Material* m = obj_inter.m;
    Vector3f N = obj_inter.normal.normalized();
    Vector3f wo = ray.direction;

    float pdf_L=1.0;
    Intersection light_inter;
    sampleLight(light_inter,pdf_L);

    Vector3f x = light_inter.coords;
    Vector3f ws = (x-p).normalized();//物体到光源 ,p->x的方向ws，也就是物体到光源的向量
    Vector3f NN = light_inter.normal.normalized();
    Vector3f emit = light_inter.emit;
    float d = (x-p).norm(); //p到光源的距离

    //再从光源发出一条光线，判断是否能打到该物体，中间是否有阻挡
    Ray Obj2Light(p,ws);
    float d2 = intersect(Obj2Light).distance;
    //没有阻挡应该距离为非负数，这里是为了弥补浮点数的误差
    if(d2-d>-0.001){
        Vector3f eval = m->eval(wo,ws,N); //BRDF值eval
      //float cos_theta = dotProduct(N,ws);
        float cos_theta = dotProduct(N,ws);
        float cos_theta_x = dotProduct(NN,-ws);//ws从物体指向光源，与NN的夹角大于180
        L_dir = emit * eval * cos_theta * cos_theta_x / std::pow(d,2) / pdf_L;

    }

     // L_indir
    float P_RR = get_random_float();
    //符合俄罗斯轮盘赌的值，就会发射光线
    if(P_RR<RussianRoulette){
        Vector3f wi = m->sample(wo,N).normalized();
        Ray r(p,wi);
        Intersection inter = intersect(r);
        // 判断打到的物体是否会发光取决于m
        if(inter.happened && !inter.m->hasEmission()){
            Vector3f eval = m->eval(wo,wi,N);
            float pdf_O = m->pdf(wo,wi,N);
            float cos_theta = dotProduct(wi,N);
            L_indir = castRay(r, depth+1) * eval * cos_theta/ pdf_O / RussianRoulette;
        }
    }
    //没用多线程 很久
    return L_dir + L_indir;
}
