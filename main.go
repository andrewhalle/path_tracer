package main

import (
	"encoding/json"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"io/ioutil"
	"math"
	"math/rand"
	"os"
	"strconv"
	"time"
)

var (
	CAMERA_POS     = vector3{0, 0, -1}
	IMAGE_TOP_LEFT = vector3{-0.5, -0.5, 0}
	IMAGE_SIZE     = 1.0
	IMAGE_RES      = 100
	PIXEL_WIDTH    = IMAGE_SIZE / float64(IMAGE_RES)
	MAX_DEPTH      = 5
	SAMPLES        = 300
	SCENE_FILENAME = "scene.json"
)

type reflectionType int

const (
	SPECULAR reflectionType = iota
	DIFFUSE
)

//- json stuff     -------------------------------------

type objType int

const (
	SPHERE objType = iota
	PLANE
)

type typedObject3D struct {
	Type objType
	Data json.RawMessage
}

func sceneFromJSON(filename string) []object3D {
	data, _ := ioutil.ReadFile(filename)
	var scene []object3D
	var typedScene []typedObject3D
	json.Unmarshal(data, &typedScene)

	for _, obj := range typedScene {
		switch obj.Type {
		case SPHERE:
			var s sphere
			json.Unmarshal(obj.Data, &s)
			scene = append(scene, s)
		case PLANE:
			var p plane
			json.Unmarshal(obj.Data, &p)
			scene = append(scene, p)
		default:
			panic("couldn't load scene from file")
		}
	}

	return scene
}

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
	X, Y, Z float64
}

func (u vector3) dot(v vector3) float64 {
	return (u.X * v.X) + (u.Y * v.Y) + (u.Z * v.Z)
}

func (u vector3) plus(v vector3) vector3 {
	return vector3{u.X + v.X, u.Y + v.Y, u.Z + v.Z}
}

func (u vector3) minus(v vector3) vector3 {
	return vector3{u.X - v.X, u.Y - v.Y, u.Z - v.Z}
}

func (u vector3) times(a float64) vector3 {
	return vector3{u.X * a, u.Y * a, u.Z * a}
}

func (u vector3) equals(v vector3) bool {
	return (u.X == v.X) && (u.Y == v.Y) && (u.Z == v.Z)
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
	return color3{c1.X * c2.X, c1.Y * c2.Y, c1.Z * c2.Z}
}

func (c1 color3) add(c2 color3) color3 {
	return color3{c1.X + c2.X, c1.Y + c2.Y, c1.Z + c2.Z}
}

func (c1 color3) scalarMultiply(a float64) color3 {
	return color3{a * c1.X, a * c1.Y, a * c1.Z}
}

func (c color3) toColor() color.RGBA {
	return color.RGBA{gammaCorrect(c.X), gammaCorrect(c.Y), gammaCorrect(c.Z), 255}
}

//- ray --------------------------------------------------

type ray struct {
	Start, Dir vector3
}

func (r ray) at(t float64) vector3 {
	return r.Start.plus(r.direction().times(t))
}

func (r ray) direction() vector3 {
	return r.Dir.norm()
}

//- plane -------------------------------------------------

type plane struct {
	Center, Normal vector3
	E, R           color3
	RT             reflectionType
}

func (p plane) intersect(r ray) (float64, bool) {
	if p.Normal.dot(r.direction()) >= 0 {
		return 0, false
	}
	t := p.Normal.dot(p.Center.minus(r.Start)) / p.Normal.dot(r.direction())
	if t < 0 {
		return 0, false
	} else {
		return t, true
	}

}

func (p plane) emittance() color3 {
	return p.E
}

func (p plane) reflectance() color3 {
	return p.R
}

func (p plane) normalAt(point vector3) vector3 {
	return p.Normal.norm()
}

func (p plane) reflType() reflectionType {
	return p.RT
}

//- sphere -----------------------------------------------

type sphere struct {
	Center vector3
	Radius float64
	E, R   color3
	RT     reflectionType
}

func (s sphere) intersect(r ray) (float64, bool) {
	a := r.direction().normSquared()
	b := 2 * r.direction().dot(r.Start.minus(s.Center))
	c := r.Start.minus(s.Center).normSquared() - math.Pow(s.Radius, 2)
	discriminant := math.Pow(b, 2) - (4 * a * c)
	var t float64
	if discriminant < 0 {
		return 0, false
	} else if discriminant == 0 {
		t = -b / (2 * a)
		if t < 0 {
			return 0, false
		}
		if r.direction().dot(s.normalAt(r.at(t))) > 0 {
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
		if r.direction().dot(s.normalAt(r.at(t))) > 0 {
			return 0, false
		}
	}
	return t, true
}

func (s sphere) emittance() color3 {
	return s.E
}

func (s sphere) reflectance() color3 {
	return s.R
}

func (s sphere) normalAt(point vector3) vector3 {
	return point.minus(s.Center).norm()
}

func (s sphere) reflType() reflectionType {
	return s.RT
}

//- object3D --------------------------------------------

type object3D interface {
	intersect(r ray) (float64, bool)
	emittance() color3
	reflectance() color3
	normalAt(point vector3) vector3
	reflType() reflectionType
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
	var newDirection vector3
	if thingHit.reflType() == DIFFUSE {
		newDirection = randomUnitVectorInHemisphereOf(thingHit.normalAt(pointHit))
	} else {
		newDirection = r.direction().minus(thingHit.normalAt(pointHit).times(2 * thingHit.normalAt(pointHit).dot(r.direction())))
	}
	newRay := ray{pointHit, newDirection}
	p := 1 / (2 * math.Pi)
	cos_theta := newDirection.dot(thingHit.normalAt(pointHit))
	brdf := thingHit.reflectance().scalarMultiply(1 / math.Pi)

	incoming := tracePath(objs, newRay, depth+1)
	return thingHit.emittance().add(brdf.multiply(incoming).scalarMultiply(cos_theta / p))
}

func parseArgs() {
	args := os.Args[1:]
	for len(args) > 0 {
		switch args[0] {
		case "--scene":
			SCENE_FILENAME = args[1]
			args = args[2:]
		case "--resolution":
			IMAGE_RES, _ = strconv.Atoi(args[1])
			PIXEL_WIDTH = IMAGE_SIZE / float64(IMAGE_RES)
			args = args[2:]
		case "--samples":
			SAMPLES, _ = strconv.Atoi(args[1])
			args = args[2:]
		default:
			panic("unable to parse args")
		}
	}
}

func main() {
	rand.Seed(time.Now().UnixNano())
	parseArgs()
	objs := sceneFromJSON(SCENE_FILENAME)
	r := image.Rect(0, 0, IMAGE_RES, IMAGE_RES)
	im := image.NewRGBA(r)
	for i := 0; i < IMAGE_RES; i++ {
		fmt.Fprintf(os.Stderr, "%d\n", i)
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
