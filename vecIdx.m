function rob_index = vecIdx(i)
%VECIDX computes a pointer to vector coordinates for each agent
%Input: agent number i; Output: 2D indices for agent i
rob_index = (2*(i-1)+1 : 2*i); %e.g. agent 1 -> vector [1,2], agent 2 -> vector [3,4]...
end