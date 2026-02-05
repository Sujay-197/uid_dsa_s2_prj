class cl1 {
    private double data = 50.2;

    cl1(double da) {
        this.data = da;
    }

    double getData() {
        return this.data;
    }
}

public class j02 {
    // access modifiers

    private int meta = 40;

    public void m2(String[] args) {
        cl1 obj = new cl1(50.2);
        System.out.println("priv Class " + obj.getData() + " meta: " + this.meta);
    }

    public static void main(String[] args) {
        j02 j = new j02();
        j.m2(args);
    }

}
