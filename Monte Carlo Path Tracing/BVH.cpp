#include "BVH.h"

// max nodes is 255
BVH::BVH(std::vector<Object*> obj, int maxPrimsInNode):
	maxPrimsInNode(std::min(255,maxPrimsInNode)),primitives(std::move(obj))
{
	if (primitives.empty()) return;

	root = recursiveBuild(primitives);
}

// object inputed is triangles not a completed object.
BVHNode* BVH::recursiveBuild(std::vector<Object*> objects)
{
	BVHNode* node = new BVHNode();
	BoundBox box;

	// union all boundboxes
	for (int i = 0; i < objects.size(); i++) {
		box = Union(box, objects[i]->getBounds());
	}

	// create leaf node if only one object left
	if (objects.size() == 1) {
		node->box = objects[0]->getBounds();
		node->object = objects[0];
		node->left = nullptr;
		node->right = nullptr;
		node->area = objects[0]->getArea(); // apply area to node
		return node;
	}
	else if (objects.size() == 2) {
		node->left = recursiveBuild(std::vector{ objects[0] });
		node->right = recursiveBuild(std::vector{ objects[1] });
		node->box = Union(node->left->box, node->right->box);
		node->area = node->left->area + node->right->area; // summing area on parent node
		return node;
	}
	else
	{
		BoundBox centroid; // box of middle bound box
		for (int i = 0; i < objects.size(); i++) 
		{
			centroid = Union(centroid, objects[i]->getBounds().Centroid());
		}
		// divide boundbox by longest axis
		int axis = centroid.longestAxis();
		switch (axis) {
			case 0: // divide by x
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().x <
								f2->getBounds().Centroid().x;
					});
				break;
			case 1:// divide by y
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().y <
						f2->getBounds().Centroid().y;
					});
				break;
			case 2:// divide by z
				std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
					return f1->getBounds().Centroid().z <
						f2->getBounds().Centroid().z;
					});
				break;
		}

		auto beginning = objects.begin();
		auto middle = objects.begin() + (objects.size() / 2);
		auto ending = objects.end();

		auto leftArray = std::vector<Object*>(beginning, middle);
		auto rightArray = std::vector<Object*>(middle, ending);

		node->left = recursiveBuild(leftArray);
		node->right = recursiveBuild(rightArray);

		node->box = Union(node->left->box, node->right->box);
		node->area = node->left->area + node->right->area;
	}

	return node;
}

Intersection BVH::Intersect(const Ray& r) const
{
	Intersection inter;
	if (!root)
		return inter;
	inter = BVH::getIntersection(root, r);
	return inter;
}

Intersection BVH::getIntersection(BVHNode* node, const Ray& r) const
{
	Intersection isect;

	std::array<int, 3> dirIsNeg;
	dirIsNeg[0] = int(r.dir.x >= 0);
	dirIsNeg[1] = int(r.dir.y >= 0);
	dirIsNeg[2] = int(r.dir.z >= 0);

	if (!node->box.IntersectPoint(r, r.dir_inverse, dirIsNeg))
	{
		return isect;
	}

	if (node->left == nullptr && node->right == nullptr)
	{
		isect = node->object->getIntersection(r);
		return isect;
	}

	auto hit1 = getIntersection(node->left, r);
	auto hit2 = getIntersection(node->right, r);

	return hit1.dist < hit2.dist ? hit1 : hit2;
}

// random sample in BVH. input a random area value[0, root->area].
// if node->area > r_area, traveres left otherwise traverse right.
void BVH::getSample(BVHNode* node, float rad_area, Intersection& inter, float& pdf)
{
	// leaf node store area of triangle while non-leaf node store the sum of its children
	// this works because when node has one child, node->area = child->area
	if (node->left == nullptr || node->right == nullptr) {
		node->object->Sample(inter, pdf);
		pdf *= node->area;
		return;
	}

	if (rad_area < node->left->area) getSample(node->left, rad_area, inter, pdf);
	else getSample(node->left, rad_area - node->left->area, inter, pdf);
}

void BVH::Sample(Intersection& inter,float& pdf) {
	float rad_area = std::sqrt(get_random_float()) * root->area;
	getSample(root, rad_area, inter, pdf);
	pdf /= root->area;
}

// not used
bool BVH::IntersectPoint(const Ray& ray) const
{
	return false;
}

