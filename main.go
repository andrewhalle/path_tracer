package main

import (
	"fmt"
	"math"
)

var (
	CAMERA_POS     = vector3{0, 0, -1}
	IMAGE_TOP_LEFT = vector3{-0.5, -0.5, 0}
	IMAGE_SIZE     = 1.0
	IMAGE_RES      = 200
)

//- util functions -------------------------------------

func minPosValue(x, y float64) (float64, bool) {
	if x < 0 && y < 0 {
		return 0, false
	} else if x < 0 && y >= 0 {
		return y, true
	} else if x >= 0 && y < 0 {
		return x, true
	} else {
		return math.Min(x, y), true
	}
}

//- vector3 ---------------------------------------------

type vector3 struct {
	x, y, z float64
}

func (u vector3) dot(v vector3) float64 {
	return (u.x * v.x) + (u.y * v.y) + (u.z * v.z)
}

func (u vector3) plus(v vector3) vector3 {
	return vector3{u.x + v.x, u.y + v.y, u.z + v.z}
}

func (u vector3) minus(v vector3) vector3 {
	return vector3{u.x - v.x, u.y - v.y, u.z - v.z}
}

func (u vector3) times(a float64) vector3 {
	return vector3{u.x * a, u.y * a, u.z * a}
}

func (u vector3) equals(v vector3) bool {
	return (u.x == v.x) && (u.y == v.y) && (u.z == v.z)
}

func (u vector3) normSquared() float64 {
	return u.dot(u)
}

//- ray --------------------------------------------------

type ray struct {
	start, direction vector3
}

func (r ray) at(t float64) vector3 {
	return r.start.plus(r.direction.times(t))
}

//- plane -------------------------------------------------

type plane struct {
	center, normal vector3
}

func (p plane) intersect(r ray) (vector3, bool) {
	if p.normal.dot(r.direction) == 0 {
		return vector3{}, 0, false
	}
	t := p.normal.dot(p.center.minus(r.start)) / p.normal.dot(r.direction)
	if t < 0 {
		return vector3{}, 0, false
	} else {
		fmt.Println(t)
		return r.at(t), t, true
	}

}

//- sphere -----------------------------------------------

type sphere struct {
	center vector3
	radius float64
}

func (s sphere) intersect(r ray) (vector3, bool) {
	a := r.direction.normSquared()
	b := 2 * r.direction.dot(r.start.minus(s.center))
	c := r.start.minus(s.center).normSquared() - math.Pow(s.radius, 2)
	discriminant := math.Pow(b, 2) - (4 * a * c)
	var t float64
	if discriminant < 0 {
		return vector3{}, 0, false
	} else if discriminant == 0 {
		t = -b / (2 * a)
		if t < 0 {
			return vector3{}, 0, false
		}
	} else {
		t1 := (-b + math.Sqrt(discriminant)) / (2 * a)
		t2 := (-b - math.Sqrt(discriminant)) / (2 * a)
		var ok bool
		t, ok = minPosValue(t1, t2)
		if !ok {
			return vector3{}, 0, false
		}
	}
	return r.at(t), t, true
}

//- object3D --------------------------------------------

type object3D interface {
	intersect(r ray) (vector3, float64, bool)
}

//- path_tracer -----------------------------------------

func main() {
	objs := []object3D{
		sphere{vector3{0, 0, 5}, 1},
	}
	fmt.Println(objs)
}
