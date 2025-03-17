return {
	"nvim-java/nvim-java",
	config = false,
	priority = 100, -- Crucial to load before lspconfig
	dependencies = {
		{ "neovim/nvim-lspconfig" },
	},
}
