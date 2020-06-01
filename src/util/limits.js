let _limits = true;

/**
 * Get whether protective limits should be imposed.
 * @param {?boolean} setTo - If given, it'll set it as well. Bad design yes, but whatever
 */
const limits = setTo => {
    if (setTo !== undefined) {
        _limits = setTo;
    }
    return _limits;
};

module.exports = limits;
