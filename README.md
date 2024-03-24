English| [简体中文](./README_cn.md)

# Environment Initialization

```
$ yarn
```

This command will install the required plugins.

# Development

```
$ yarn start
```

This command starts the local development server and opens a browser window. Most changes are reflected in real-time without the need to restart the server.

# Compilation

```
$ yarn build
```

This command generates static content in the "build" directory, which can be used for any static content hosting service. After compilation, it is not supported to directly open the .html file for viewing; you need to use `npm run serve` to start the service.

# Error Handling
1. If initialization of the environment using `yarn` fails, try upgrading yarn to the latest version
```
npm install yarn@latest -g
```
2. If starting the local development server with `yarn start` fails, try upgrading the nodejs version
```
# Install the n module
sudo npm install -g n

# Upgrade nodejs to the specified version
sudo n node_version_number

# Check the nodejs version (you might need to restart the terminal)
node -v
```