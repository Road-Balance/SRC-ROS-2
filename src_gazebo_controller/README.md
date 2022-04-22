# Ackermann Steering Car Controller dynamics

| Convert `cmd_vel` type msg to `Ackermann Steering Control signal`

Ackermann Steering Robot in this example consists of...
- Each Wheel can controlled separately (So it means differential gear in real.)
- Front Wheels can steered with revolute hinge joints 

## Step 1. Let's start from basic bycicle model dynamics

We can calculate steering radius && wheel rotational velocity for COM

![슬라이드2](https://user-images.githubusercontent.com/12381733/164608710-81e349d8-7ba0-4451-b133-07cdf30ec50d.png)

## Step 2. Calculate rear wheel velocity with different steering radius

![슬라이드3](https://user-images.githubusercontent.com/12381733/164608714-075d64fd-afc7-406a-b378-9301993a8de2.png)

## Step 3. Calculate front wheel velocity & steering angle alpha for each joints

There's lots of equations for this `alpha`. I tried all of them and found best one empirically.

![슬라이드4](https://user-images.githubusercontent.com/12381733/164608718-1df9c453-3a46-43a2-805c-d166a0311bff.png)

