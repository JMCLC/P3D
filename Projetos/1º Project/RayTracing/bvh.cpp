#include "rayAccelerator.h"
#include "macros.h"

using namespace std;


BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object*>& objs) {


	BVHNode* root = new BVHNode();

	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB world_bbox = AABB(min, max);

	for (Object* obj : objs) {
		AABB bbox = obj->GetBoundingBox();
		world_bbox.extend(bbox);
		objects.push_back(obj);
	}
	world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
	world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
	root->setAABB(world_bbox);
	nodes.push_back(root);
	build_recursive(0, objects.size(), root); // -> root node takes all the 
}

int maxAxis = 0;
void BVH::build_recursive(int left_index, int right_index, BVHNode* node) {

	int n_objs = right_index - left_index;

	if (n_objs <= Threshold) {
		node->makeLeaf(left_index, n_objs);
		return;
	}

	int split_index = left_index;
	float mid_point;

	AABB worldbb = node->getAABB();
	float mid_x = worldbb.max.x - worldbb.min.x;
	float mid_y = worldbb.max.y - worldbb.min.y;
	float mid_z = worldbb.max.z - worldbb.min.z;

	if (mid_x >= mid_z) {
		mid_point = mid_x;
		maxAxis = 0;
	}
	else
	{
		mid_point = mid_y;
		maxAxis = 1;
	}

	if (mid_z > mid_point)
	{
		mid_point = mid_z;
		maxAxis = 2;
	}

	mid_point /= 2.0 + worldbb.min.getAxisValue(maxAxis);

	Comparator cmp = Comparator();
	cmp.dimension = maxAxis;

	std::sort(objects.begin()+left_index, objects.begin()+right_index, cmp);

	if (objects.at(left_index)->getCentroid().getAxisValue(maxAxis) > mid_point ||
		objects.at(right_index - 1)->getCentroid().getAxisValue(maxAxis) <= mid_point)
	{
		split_index = (left_index + right_index) / 2;
	}
	else {
		for (split_index = left_index; split_index < right_index; split_index++)
		{
			if (objects.at(split_index)->getCentroid().getAxisValue(maxAxis) > mid_point)
			{
				break;
			}
		}
	}

	AABB leftBoundingBox = AABB(Vector(FLT_MAX, FLT_MAX, FLT_MAX), Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX));
	AABB rightBoundingBox = AABB(Vector(FLT_MAX, FLT_MAX, FLT_MAX), Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX));

	for (int i = left_index; i < split_index; i++)
	{
		AABB box = objects.at(i)->GetBoundingBox();
		leftBoundingBox.extend(box);
	}

	for (int i = split_index; i < right_index; i++)
	{
		AABB box = objects.at(i)->GetBoundingBox();
		rightBoundingBox.extend(box);
	}

	BVHNode* leftChild = new BVHNode();
	BVHNode* rightChild = new BVHNode();

	node->makeNode(nodes.size());

	leftChild->setAABB(leftBoundingBox);
	rightChild->setAABB(rightBoundingBox);
	nodes.push_back(leftChild);
	nodes.push_back(rightChild);

	build_recursive(left_index, split_index, leftChild);
	build_recursive(split_index, right_index, rightChild);

}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
	float t, tmin = FLT_MAX;
	Object* closest_obj = nullptr;

	BVHNode* curr_node = nodes[0];

	if (!curr_node->getAABB().intercepts(ray, t)) {
		closest_obj = nullptr;
	}

	while (true) {

		if (curr_node->isLeaf()) {
			int base_index = curr_node->getIndex(), size = curr_node->getNObjs();

			for (int i = base_index; i < base_index + size; i++) {
				bool inter = objects[i]->intercepts(ray, t);

				if (inter && t < tmin) {
					tmin = t;
					closest_obj = objects[i];
				}
			}
		}
		else {

			float t1 = FLT_MAX;
			float t2 = FLT_MAX;
			BVHNode* child1 = nodes[curr_node->getIndex()];
			BVHNode* child2 = nodes[curr_node->getIndex() + 1];

			bool c1 = child1->getAABB().intercepts(ray, t1);
			bool c2 = child2->getAABB().intercepts(ray, t2);

			if (child1->getAABB().isInside(ray.origin)) t1 = 0;
			if (child2->getAABB().isInside(ray.origin)) t2 = 0;

			StackItem* stack = nullptr;

			if (c1 && c2) {
				if (t2 < t1) {
					curr_node = child2;
					stack = new StackItem(child1, t1);
				}
				else {
					curr_node = child1;
					stack = new StackItem(child2, t2);
				}

				hit_stack.push(*stack);
				continue;
			}
			else if (c1) {
				curr_node = child1;
				continue;
			}
			else if (c2) {
				curr_node = child2;
				continue;
			}
		}

		while (true) {
			if (hit_stack.empty()) {
				*hit_obj = closest_obj;
				if (*hit_obj == nullptr) {
					return false;
				}
				hit_point = ray.origin + ray.direction * tmin;
				return true;
			}
			else {
				StackItem stack = hit_stack.top();
				hit_stack.pop();

				if (stack.t < tmin) {
					curr_node = stack.ptr;
					break;
				}
			}
		}
	}
}


bool BVH::Traverse(Ray& ray) {
	float t;
	float max_t = ray.direction.length();
	Object* closest_obj = nullptr;
	BVHNode* curr_node = nodes[0];

	if (!curr_node->getAABB().intercepts(ray, t))
	{
		return false;
	}

	while (true)
	{
		if (curr_node->isLeaf())
		{
			int base_index = curr_node->getIndex(), size = curr_node->getNObjs();
			for (int i = base_index; i < base_index + size; i++)
			{
				if (objects[i]->intercepts(ray, t) && t < max_t)
				{
					return true;
				}
			}
		}
		else
		{
			float t1 = FLT_MAX;
			float t2 = FLT_MAX;
			BVHNode* child1 = nodes[curr_node->getIndex()];
			BVHNode* child2 = nodes[curr_node->getIndex() + 1];

			bool c1 = child1->getAABB().intercepts(ray, t1);
			bool c2 = child2->getAABB().intercepts(ray, t2);

			StackItem* stack = nullptr;

			if (c1 && c2)
			{
				curr_node = child1;
				stack = new StackItem(child2, t2);
				hit_stack.push(*stack);
				continue;
			}
			else if (c1)
			{
				curr_node = child1;
				continue;
			}
			else if (c2)
			{
				curr_node = child2;
				continue;
			}
		}

		if (hit_stack.empty())
		{
			return false;
		}
		else
		{
			StackItem stack = hit_stack.top();
			hit_stack.pop();
			curr_node = stack.ptr;
		}
	}
}
