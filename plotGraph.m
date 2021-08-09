figure(2)
plot(1:t, PL(1:t, 1))
hold on
plot(1:t, P(1:t, 1, 1))
plot(1:t, P(1:t, 1, 2))
plot(1:t, P(1:t, 1, 3))
plot(1:t, P(1:t, 1, 4))

legend('virtual Leader', 'robot 1', 'robot 2', 'robot 3', 'robot 4')

hold off 
figure(3)
plot(1:t, PL(1:t, 2))
hold on
plot(1:t, P( 1:t, 2, 1))
plot(1:t, P( 1:t, 2, 2))
plot(1:t, P(1:t, 2, 3))
plot(1:t, P(1:t, 2, 4))
legend('virtual Leader', 'robot 1', 'robot 2', 'robot 3', 'robot 4')

hold off

figure(4)
plot(1:t, P( 1:t, 3, 1))
hold on
plot(1:t, P(1:t, 3, 2))
plot(1:t, P(1:t, 3, 3))
plot(1:t, P(1:t, 3, 4))
hold off


figure(5)

plot(1:t, V(1, 1:t, 4, 1))
hold on
plot(1:t, V(2, 1:t, 4, 1))
plot(1:t, V(3, 1:t, 4, 1))
plot(1:t, V(4, 1:t, 4, 1))
hold off
figure(6)

plot(1:t, V(1, 1:t, 1, 1))
hold on
plot(1:t, V(2, 1:t, 1, 1))
plot(1:t, V(3, 1:t, 1, 1))
plot(1:t, V(4, 1:t, 1, 1))

hold off
figure(7)
plot(1:t, V(1, 1:t, 2, 1))
hold on
plot(1:t, V(2, 1:t, 2, 1))
plot(1:t, V(3, 1:t, 2, 1))
plot(1:t, V(4, 1:t, 2, 1))
hold off

figure(8)
plot(1:t, V(1, 1:t, 3, 1))
hold on
plot(1:t, V(2, 1:t, 3, 1))
plot(1:t, V(3, 1:t, 3, 1))
plot(1:t, V(4, 1:t, 3, 1))

hold off
figure(9)

plot(1:t, V(1, 1:t, 4, 2))
hold on
plot(1:t, V(2, 1:t, 4, 2))
plot(1:t, V(3, 1:t, 4, 2))
plot(1:t, V(4, 1:t, 4, 2))

hold off

figure(10)

plot(1:t, V(1, 1:t, 1, 2))
hold on
plot(1:t, V(2, 1:t, 1, 2))
plot(1:t, V(3, 1:t, 1, 2))
plot(1:t, V(4, 1:t, 1, 2))

hold off
figure(11)
plot(1:t, V(1, 1:t, 2, 2))
hold on
plot(1:t, V(2, 1:t, 2, 2))
plot(1:t, V(3, 1:t, 2, 2))
plot(1:t, V(4, 1:t, 2, 2))

hold off
figure(12)

plot(1:t, V(1, 1:t, 3, 2))
hold on
plot(1:t, V(2, 1:t, 3, 2))
plot(1:t, V(3, 1:t, 3, 2))
plot(1:t, V(4, 1:t, 3, 2))
hold off

VL=zeros(NUM_ROBOTS, t, 4);
for i=1:NUM_ROBOTS
    for j=1:t
        for k=1:4
            VL(i, j, k)=vectorLength(V(i, j, k, :));
        end
    end
end


figure(13)

plot(1:t, VL(1, :, 4))
hold on
plot(1:t, VL(2, :, 4))
plot(1:t, VL(3, :, 4))
plot(1:t, VL(4, :, 4))
legend( 'robot 1', 'robot 2', 'robot 3', 'robot 4')
xlabel('t - время (шаг)')
ylabel('Скорость')

hold off
figure(14)
plot(1:t, VL(1, :, 1))
hold on
plot(1:t, VL(2, :, 1))
plot(1:t, VL(3, :, 1))
plot(1:t, VL(4, :, 1))
legend( 'robot 1', 'robot 2', 'robot 3', 'robot 4')
xlabel('t - время (шаг)')
ylabel('Скорость избегания преграды')
hold off
figure(15)
plot(1:t, VL(1, :, 2))
hold on
plot(1:t, VL(2, :, 2))
plot(1:t, VL(3, :, 2))
plot(1:t, VL(4, :, 2))

legend( 'robot 1', 'robot 2', 'robot 3', 'robot 4')
xlabel('t - время (шаг)')
ylabel('Скорость сохранения строении')

hold off
figure(16)
plot(1:t, VL(1, :, 3))
hold on
plot(1:t, VL(2, :, 3))
plot(1:t, VL(3, :, 3))
plot(1:t, VL(4, :, 3))
legend( 'robot 1', 'robot 2', 'robot 3', 'robot 4')
xlabel('t - время (шаг)')
ylabel('Скорость движения к цели')
hold off

VA=zeros(NUM_ROBOTS, t, 4);
for i=1:NUM_ROBOTS
    for j=1:t
        for k=1:4
            VA(i, j, k)=vectorOrientation(V(i, j, k, :));
        end
    end
end


figure(17)

plot(1:t, VA(1, :, 4))
hold on
plot(1:t, VA(2, :, 4))
plot(1:t, VA(3, :, 4))
plot(1:t, VA(4, :, 4))
hold off
figure(18)
plot(1:t, VA(1, :, 1))
hold on
plot(1:t, VA(2, :, 1))
plot(1:t, VA(3, :, 1))
plot(1:t, VA(4, :, 1))

hold off
figure(19)
plot(1:t, VA(1, :, 2))
hold on
plot(1:t, VA(2, :, 2))
plot(1:t, VA(3, :, 2))
plot(1:t, VA(4, :, 2))

hold off
figure(20)
plot(1:t, VA(1, :, 3))
hold on
plot(1:t, VA(2, :, 3))
plot(1:t, VA(3, :, 3))
plot(1:t, VA(4, :, 3))

hold off