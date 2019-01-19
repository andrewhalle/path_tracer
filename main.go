package main

import (
	"fmt"
	"image"
	"image/color"
	"image/png"
	"math"
	"math/rand"
	"os"
	"time"
)

var (
	CAMERA_POS     = vector3{0, 0, -1}
	IMAGE_TOP_LEFT = vector3{-0.5, -0.5, 0}
	IMAGE_SIZE     = 1.0
	IMAGE_RES      = 200
	PIXEL_WIDTH    = IMAGE_SIZE / float64(IMAGE_RES)
	MAX_DEPTH      = 3
	SAMPLES        = 10000
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

func randomUnitVectorInHemisphereOf(norm vector3) vector3 {
	for {
		theta := 2 * math.Pi * rand.Float64()
		z := (rand.Float64() * 2) - 1
		curr := vector3{math.Sqrt(1-math.Pow(z, 2)) * math.Cos(theta), math.Sqrt(1-math.Pow(z, 2)) * math.Sin(theta), z}
		if curr.dot(norm) >= 0 {
			return curr
		}
	}

}

func gammaCorrect(x float64) uint8 {
	return uint8(math.Pow(clamp(x), 1/2.2) * 255)
}

func clamp(x float64) float64 {
	if x < 0 {
		return 0
	} else if x > 1 {
		return 1
	} else {
		return x
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

func (u vector3) norm() vector3 {
	return u.times(math.Sqrt(u.normSquared()))
}

//- color3 -----------------------------------------------

type color3 vector3

func (c1 color3) multiply(c2 color3) color3 {
	return color3{c1.x * c2.x, c1.y * c2.y, c1.z * c2.z}
}

func (c1 color3) add(c2 color3) color3 {
	return color3{c1.x + c2.x, c1.y + c2.y, c1.z + c2.z}
}

func (c1 color3) scalarMultiply(a float64) color3 {
	return color3{a * c1.x, a * c1.y, a * c1.z}
}

func (c color3) toColor() color.RGBA {
	return color.RGBA{gammaCorrect(c.x), gammaCorrect(c.y), gammaCorrect(c.z), 255}
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
	e, r           color3
}

func (p plane) intersect(r ray) (float64, bool) {
	if p.normal.dot(r.direction) == 0 {
		return 0, false
	}
	t := p.normal.dot(p.center.minus(r.start)) / p.normal.dot(r.direction)
	if t < 0 {
		return 0, false
	} else {
		fmt.Println(t)
		return t, true
	}

}

func (p plane) emittance() color3 {
	return p.e
}

func (p plane) reflectance() color3 {
	return p.r
}

func (p plane) normalAt(point vector3) vector3 {
	return p.normal.norm()
}

//- sphere -----------------------------------------------

type sphere struct {
	center vector3
	radius float64
	e, r   color3
}

func (s sphere) intersect(r ray) (float64, bool) {
	a := r.direction.normSquared()
	b := 2 * r.direction.dot(r.start.minus(s.center))
	c := r.start.minus(s.center).normSquared() - math.Pow(s.radius, 2)
	discriminant := math.Pow(b, 2) - (4 * a * c)
	var t float64
	if discriminant < 0 {
		return 0, false
	} else if discriminant == 0 {
		t = -b / (2 * a)
		if t < 0 {
			return 0, false
		}
	} else {
		t1 := (-b + math.Sqrt(discriminant)) / (2 * a)
		t2 := (-b - math.Sqrt(discriminant)) / (2 * a)
		var ok bool
		t, ok = minPosValue(t1, t2)
		if !ok {
			return 0, false
		}
	}
	return t, true
}

func (s sphere) emittance() color3 {
	return s.e
}

func (s sphere) reflectance() color3 {
	return s.r
}

func (s sphere) normalAt(point vector3) vector3 {
	return point.minus(s.center).norm()
}

//- object3D --------------------------------------------

type object3D interface {
	intersect(r ray) (float64, bool)
	emittance() color3
	reflectance() color3
	normalAt(point vector3) vector3
}

//- path_tracer -----------------------------------------

func tracePath(objs []object3D, r ray, depth int) color3 {
	if depth >= MAX_DEPTH {
		return color3{}
	}
	minT, found := math.Inf(1), false
	var thingHit object3D
	for _, obj := range objs {
		t, ok := obj.intersect(r)
		if ok && t < minT {
			minT = t
			thingHit = obj
			found = true
		}
	}
	if !found {
		return color3{}
	}
	pointHit := r.at(minT)
	newDirection := randomUnitVectorInHemisphereOf(thingHit.normalAt(pointHit))
	newRay := ray{pointHit, newDirection}
	p := 1 / (2 * math.Pi)
	cos_theta := newDirection.dot(thingHit.normalAt(pointHit))
	brdf := thingHit.reflectance().scalarMultiply(1 / math.Pi)

	incoming := tracePath(objs, newRay, depth+1)
	return thingHit.emittance().add(brdf.multiply(incoming).scalarMultiply(cos_theta / p))
}

func main() {
	rand.Seed(time.Now().UnixNano())
	objs := []object3D{
		sphere{vector3{-1, -1, 5},
			1,
			color3{},
			color3{0.5, 0.25, 0.125},
		},
		sphere{vector3{1, 1, 5},
			1,
			color3{0.9, 0.9, 0.9},
			color3{},
		},
	}
	r := image.Rect(0, 0, IMAGE_RES, IMAGE_RES)
	im := image.NewRGBA(r)
	for i := 0; i < IMAGE_RES; i++ {
		for j := 0; j < IMAGE_RES; j++ {
			finalColor := color3{}
			for n := 0; n < SAMPLES; n++ {
				pixelPos := IMAGE_TOP_LEFT.plus(vector3{1, 0, 0}.times(PIXEL_WIDTH * float64(i))).plus(vector3{0, 1, 0}.times(PIXEL_WIDTH * float64(j)))
				r := ray{CAMERA_POS, pixelPos.minus(CAMERA_POS).norm()}
				c := tracePath(objs, r, 0)
				finalColor = finalColor.add(c)
			}
			finalColor = finalColor.scalarMultiply(1 / float64(SAMPLES))
			im.SetRGBA(i, j, finalColor.toColor())
		}
	}
	fmt.Println(png.Encode(os.Stdout, im))
}
