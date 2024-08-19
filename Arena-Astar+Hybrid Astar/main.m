clear all;
clc;
close;

abspath= utils('abspath');
env = Env(abspath('sys.ini'));
policy = Policy();
% policy = Policy_t();
if (env.succeed)
    observation = env.reset();
    while 1
        env.render();
        action = policy.action(observation);
        
        [observation,done,info] = env.step(action);
        
        disp(info);
        if(done)
            break;
        end
        wait(1);
    end
end



function wait(ms)
time = ms/1000;
% tic
pause(time)
% toc
end

