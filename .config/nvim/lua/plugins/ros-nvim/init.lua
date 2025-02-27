return {
	"tadachs/ros-nvim",
	dependencies = { "nvim-lua/plenary.nvim" },
	opts = function()
		local ret = {
			setup = {
				only_workspace = true,
			},
		}
		return ret
	end,
}
