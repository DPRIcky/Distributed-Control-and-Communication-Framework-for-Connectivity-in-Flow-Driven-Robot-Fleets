function [V, LfV] = compute_clf(x, xg)
    V = 0.5 * norm(x - xg)^2;
    LfV = (x - xg)';
end
