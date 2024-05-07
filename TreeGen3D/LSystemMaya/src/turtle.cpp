#include <iostream>
#include "turtle.h"

Turtle::Turtle()
	:position(0, 0, 0)
	, direction(0, 1, 0)
	, right(1, 0, 0)
	, thickness(1)
	, reduction(1.0)
	, scalar(1.0)
	, class_id(0)
	, iter_num(1)
	, branch_status(0)
	, strahler_number(0)
{

	int fff1 = 0;
}


void Turtle::setReduction(float param)
{
	reduction = param / 100;
}
void Turtle::setThickness(float param)
{
	thickness *= param;
}
void Turtle::thicken(float param)
{
	thickness += thickness * param / 100;
}
void Turtle::narrow(float param)
{
	thicken(-param);
}
void Turtle::turnRight(float angle)
{
	angle = angle * M_PI / 180;
	R3Vector axis = direction;
	axis.Cross(right);
	direction.Rotate(axis, angle);
	right.Rotate(axis, angle);
	direction.Normalize();
	right.Normalize();
}
void Turtle::turnLeft(float angle)
{
	turnRight(-angle);
}
void Turtle::pitchUp(float angle)
{
	angle = angle * M_PI / 180;
	direction.Rotate(right, angle);
	direction.Normalize();
}
void Turtle::pitchDown(float angle)
{
	pitchUp(-angle);
}
void Turtle::rollRight(float angle)
{
	angle = angle * M_PI / 180;
	right.Rotate(direction, angle);
	right.Normalize();
}
void Turtle::rollLeft(float angle)
{
	rollRight(-angle);
}
void Turtle::move(float distance)
{
	R3Vector t = direction;
	t.Normalize();
	position += scalar * t;
}
void Turtle::turn180(float temp)
{
	turnRight(M_PI);
}

void Turtle::setScale(float param) {
	float param2 = param + (randInt(0, 4) / 5.0 - 0.4);
	scalar *= param;
}

void Turtle::setCalssID(int param) {
	class_id = param;
}

void Turtle::set_strahler_number(int param) {
	strahler_number = param;
}

TurtleSystem::TurtleSystem()
{
	group_id = 0;
	use_random_rule = false;
}

void TurtleSystem::save()
{
	Turtle t = (Turtle)*this;
	state.push(t);
}
void TurtleSystem::restore()
{
	Turtle t = state.top();

	state.pop();

	position = t.position;
	direction = t.direction;
	right = t.right;
	thickness = t.thickness;
	reduction = t.reduction;
	scalar = t.scalar;
	class_id = t.class_id;
	iter_num = t.iter_num;
	branch_status = t.branch_status;
	strahler_number = t.strahler_number;
}

void TurtleSystem::save_scalar()
{
	state_scalar.push(scalar);
}
void TurtleSystem::restore_scalar()
{
	float s = state_scalar.top();
	state_scalar.pop();
	scalar = s;
}

void TurtleSystem::save_thickness() {
	state_thickness.push(thickness);
}
void TurtleSystem::restore_thickness() {
	float s = state_thickness.top();
	state_thickness.pop();
	thickness = s;
}

void TurtleSystem::add_group_id() {
	group_id += 1;
}

void TurtleSystem::add_iter_num(int offset) {
	iter_num += offset;
}
void TurtleSystem::set_branch_status(int indicator) {
	branch_status = indicator;
}

void TurtleSystem::save_iter_num() {
	state_iter_num.push(iter_num);
}
void TurtleSystem::restore_iter_num() {
	float s = state_iter_num.top();
	state_iter_num.pop();
	iter_num = s;
}

void TurtleSystem::draw(float param)
{
	static int num = 0;
	if (num++ % 1000 == 0) cout << num << " drawing" << endl;

	R2Vector pstart(position.X(), position.Y());
	R2Vector dir2(direction.X(), direction.Y());

	R2Vector pend = pstart + scalar * dir2;

	t_line cur_line(pstart, pend, group_id, class_id, dir2, thickness);
	cur_line.branch_status = branch_status;
	cur_line.iteration = iter_num;
	cur_line.strahler_number = strahler_number;
	tree2d.push_back(cur_line);

	add_group_id();
}

void TurtleSystem::clear() {
	position = R3Vector(0, 0, 0);
	direction = R3Vector(0, 1, 0);
	right = R3Vector(1, 0, 0);
	scalar = 1.0;
	tree2d.clear();
	group_id = 0;
	class_id = 0;
	thickness = 1.0;
	iter_num = 1;
	branch_status = 0;
}
