// inheritance example

class Gun {
    boolean NeedsPermit = true;

    void shoot() {
        System.out.println("Kaboom Kaplow");
    }
}

class Sniper extends Gun {
    int weight = 10;
}

public class j03 {
    public static void main(String[] args) {
        Sniper s = new Sniper();
        s.shoot();
        System.out.println("parent property - Sniper (Gun) needs permit: " + s.NeedsPermit);
        System.out.println("child mutation - Sniper weight: " + s.weight);
    }
}
