// package main is a module with an ackermann pwm base component
package main

import (
	"context"

	"github.com/edaniels/golog"
	"github.com/viam-labs/ackermann-pwm-base/pwmbase"
	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/module"
	"go.viam.com/utils"
)

func main() {
	utils.ContextualMain(mainWithArgs, golog.NewDevelopmentLogger("ackermann-pwm-base"))
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	ackermannBase, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}

	ackermannBase.AddModelFromRegistry(ctx, base.API, pwmbase.Model)

	err = ackermannBase.Start(ctx)
	defer ackermannBase.Close(ctx)
	if err != nil {
		return err
	}

	<-ctx.Done()
	return nil
}
