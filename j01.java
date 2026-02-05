class Circle {
    double radius; // Declare instance variable

    // Constructor
    Circle(double rad) {
        this.radius = rad; // rad is local variable
    }

    // Option 1: Non-static method (for object-specific data) - uses instance
    // variables
    double _area() {
        return 3.14 * this.radius * this.radius;
    }

    // Option 2: Static method (requires a parameter) - > this is used when function
    // method is independent of object data -> class specific not object specific
    static double _circumference(double r) {
        return 3.14 * 2 * r;
    }
}

public class j01 {
    public static void main(String[] args) {
        Circle c1 = new Circle(5);

        System.out.println("Hello, World!");

        // Calling non-static method on the object
        System.out.println("Area (instance): " + c1._area());

        // Calling static method using the class name
        System.out.println("Circumference (static): " + Circle._circumference(2));
    }
}
